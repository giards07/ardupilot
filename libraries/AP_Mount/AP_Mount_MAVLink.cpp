// -*- tab-width: 4; Mode: C++; c-basic-offset: 4; indent-tabs-mode: nil -*-

#include <AP_Mount_MAVLink.h>
#if AP_AHRS_NAVEKF_AVAILABLE
#include <GCS_MAVLink.h>

#define MOUNT_DEBUG 1

#if MOUNT_DEBUG
#include <stdio.h>
#endif

AP_Mount_MAVLink::AP_Mount_MAVLink(AP_Mount &frontend, AP_Mount::mount_state &state, uint8_t instance) :
    AP_Mount_Backend(frontend, state, instance),
    _initialised(false),
    _ekf(frontend._ahrs),
    vehicleYawRateDem(0.0f)
{}

// init - performs any required initialisation for this instance
void AP_Mount_MAVLink::init(const AP_SerialManager& serial_manager)
{
    _initialised = true;
    set_mode((enum MAV_MOUNT_MODE)_state._default_mode.get());
}

// update mount position - should be called periodically
void AP_Mount_MAVLink::update()
{
    // exit immediately if not initialised
    if (!_initialised) {
        return;
    }

    // update based on mount mode
    switch(get_mode()) {
    // move mount to a "retracted" position.  we do not implement a separate servo based retract mechanism
    case MAV_MOUNT_MODE_RETRACT:
        break;

        // move mount to a neutral position, typically pointing forward
    case MAV_MOUNT_MODE_NEUTRAL:
        break;

        // point to the angles given by a mavlink message
    case MAV_MOUNT_MODE_MAVLINK_TARGETING:
        // do nothing because earth-frame angle targets (i.e. _angle_ef_target_rad) should have already been set by a MOUNT_CONTROL message from GCS
        break;

        // RC radio manual angle control, but with stabilization from the AHRS
    case MAV_MOUNT_MODE_RC_TARGETING:
        // update targets using pilot's rc inputs
        update_targets_from_rc();
        break;

        // point mount to a GPS point given by the mission planner
    case MAV_MOUNT_MODE_GPS_POINT:
        if(_frontend._ahrs.get_gps().status() >= AP_GPS::GPS_OK_FIX_2D) {
            calc_angle_to_location(_state._roi_target, _angle_ef_target_rad, true, true);
        }
        break;

    default:
        // we do not know this mode so do nothing
        break;
    }
}

// has_pan_control - returns true if this mount can control it's pan (required for multicopters)
bool AP_Mount_MAVLink::has_pan_control() const
{
    // we do not have yaw control
    return false;
}

// set_mode - sets mount's mode
void AP_Mount_MAVLink::set_mode(enum MAV_MOUNT_MODE mode)
{
    // exit immediately if not initialised
    if (!_initialised) {
        return;
    }

    // record the mode change
    _state._mode = mode;
}

// status_msg - called to allow mounts to send their status to GCS using the MOUNT_STATUS message
void AP_Mount_MAVLink::status_msg(mavlink_channel_t chan)
{
    // do nothing - we rely on the mount sending the messages directly
}


/*
  handle a GIMBAL_REPORT message
 */
void AP_Mount_MAVLink::handle_gimbal_report(mavlink_channel_t chan, mavlink_message_t *msg)
{
    // just save it for future processing and reporting to GCS for now
    mavlink_msg_gimbal_report_decode(msg, &_gimbal_report);

    Vector3f delta_angles(_gimbal_report.delta_angle_x,
                          _gimbal_report.delta_angle_y,
                          _gimbal_report.delta_angle_z);
    Vector3f delta_velocity(_gimbal_report.delta_velocity_x,
                            _gimbal_report.delta_velocity_y,
                            _gimbal_report.delta_velocity_z);
    Vector3f joint_angles(_gimbal_report.joint_roll,
                          _gimbal_report.joint_pitch,
                          _gimbal_report.joint_yaw);

    // Run the gimbal attitude and gyro bias estimator
    _ekf.RunEKF(_gimbal_report.delta_time, delta_angles, delta_velocity, joint_angles);

    // Don't send rate demands to the gimbal until the estimator has completed alignment
    if (!_ekf.getStatus()) return;

    // get the gyro bias data
    Vector3f gyroBias;
    _ekf.getGyroBias(gyroBias);

    // get the gimbal quaternion estimate
    Quaternion quatEst;
    _ekf.getQuat(quatEst);

    // The gimbal control has two modes: earth relative pointing which is the default and a vehicle relative neutral position
    // We calculate a gimbal rate vector demand and a vehicle yaw rate demand
    Vector3f gimbalRateDemVec;
    if (_state._mode == MAV_MOUNT_MODE_NEUTRAL) { // align gimbal with vehicle axes

        // Define rotation from vehicle to gimbal using a 312 rotation sequence
        Matrix3f Tvg;
        float cosPhi = cosf(joint_angles.x);
        float cosTheta = cosf(joint_angles.y);
        float sinPhi = sinf(joint_angles.x);
        float sinTheta = sinf(joint_angles.y);
        float sinPsi = sinf(joint_angles.z);
        float cosPsi = cosf(joint_angles.z);
        Tvg[0][0] = cosTheta*cosPsi-sinPsi*sinPhi*sinTheta;
        Tvg[1][0] = -sinPsi*cosPhi;
        Tvg[2][0] = cosPsi*sinTheta+cosTheta*sinPsi*sinPhi;
        Tvg[0][1] = cosTheta*sinPsi+cosPsi*sinPhi*sinTheta;
        Tvg[1][1] = cosPsi*cosPhi;
        Tvg[2][1] = sinPsi*sinTheta-cosTheta*cosPsi*sinPhi;
        Tvg[0][2] = -sinTheta*cosPhi;
        Tvg[1][2] = sinPhi;
        Tvg[2][2] = cosTheta*cosPhi;

        // convert vehicle to gimbal rotation matrix to a rotation vector using small angle approximation
        Vector3f deltaAngErr;
        deltaAngErr.x = (Tvg[2][1] - Tvg[1][2]) * 0.5f;
        deltaAngErr.y = (Tvg[0][2] - Tvg[2][0]) * 0.5f;
        deltaAngErr.z = (Tvg[1][0] - Tvg[0][1]) * 0.5f;

        // multiply the rotation vector by an error gain to calculate a demanded vehicle frame relative rate vector
        gimbalRateDemVec = deltaAngErr * K_gimbalRate;

        // constrain the vehicle relative rate vector length
        float length = gimbalRateDemVec.length();
        if (length > angRateLimit) {
            gimbalRateDemVec = gimbalRateDemVec * (angRateLimit / length);
        }

        // rotate the earth relative vehicle angular rate vector into gimbal axes and add to obtain the demanded gimbal angular rate
        gimbalRateDemVec += Tvg * _frontend._ahrs.get_gyro();

    } else { // align gimbal to earth relative demands

        // Calculate the demanded quaternion orientation for the gimbal
        // set the demanded quaternion using demanded 321 Euler angles wrt earth frame
        Quaternion quatDem;
        quatDem.from_euler(_angle_ef_target_rad.x,
                           _angle_ef_target_rad.y,
                           _angle_ef_target_rad.z);

        //divide the demanded quaternion by the estimated to get the error
        Quaternion quatErr = quatDem / quatEst;

        // convert the quaternion to an angle error vector
        Vector3f deltaAngErr;
        float scaler = 1.0f-quatErr[0]*quatErr[0];
        if (scaler > 1e-12) {
            scaler = 1.0f/sqrtf(scaler);
            if (quatErr[0] < 0.0f) {
                scaler *= -1.0f;
            }
            deltaAngErr.x = quatErr[1] * scaler;
            deltaAngErr.y = quatErr[2] * scaler;
            deltaAngErr.z = quatErr[3] * scaler;
        } else {
            deltaAngErr.zero();
        }

        // multiply the angle error vector by a gain to calculate a demanded gimbal rate
        gimbalRateDemVec = deltaAngErr * K_gimbalRate;

        // Constrain the demanded rate
        float length = gimbalRateDemVec.length();
        if (length > angRateLimit) {
            gimbalRateDemVec = gimbalRateDemVec * (angRateLimit / length);
        }

    }

    // send the gimbal control message
    mavlink_msg_gimbal_control_send(chan,
                                    msg->sysid,
                                    msg->compid,
                                    gimbalRateDemVec.x, gimbalRateDemVec.y, gimbalRateDemVec.z, // demanded rates
                                    gyroBias.x, gyroBias.y, gyroBias.z);

    // calculate the sensor to NED cosine matrix and use the last row to calcuate the earth frame z component
    // this is the vehicle yaw rate required to keep the vehicle and gimbal rotating at the same rate
    Matrix3f Tsn;
    quatEst.rotation_matrix(Tsn);
    vehicleYawRateDem = Tsn.c.x * gimbalRateDemVec.x + Tsn.c.y * gimbalRateDemVec.y + Tsn.c.z * gimbalRateDemVec.z;

    // correct the vehicle yaw rate demand to keep the yaw gimbal joint centred relative to mechanical and visual limits
    vehicleYawRateDem += (_gimbal_report.joint_yaw - gimbalYawOffset) * K_vehicleRate;

    // constrain the vehicle yaw rate demand
    vehicleYawRateDem = constrain_float(vehicleYawRateDem, -vehYawRateLim, vehYawRateLim);

}

/*
  send a GIMBAL_REPORT message to the GCS
 */
void AP_Mount_MAVLink::send_gimbal_report(mavlink_channel_t chan)
{
    mavlink_msg_gimbal_report_send(chan,
                                   0, 0, // send as broadcast
                                   _gimbal_report.delta_time,
                                   _gimbal_report.delta_angle_x,
                                   _gimbal_report.delta_angle_y,
                                   _gimbal_report.delta_angle_z,
                                   _gimbal_report.delta_velocity_x,
                                   _gimbal_report.delta_velocity_y,
                                   _gimbal_report.delta_velocity_z,
                                   _gimbal_report.joint_roll,
                                   _gimbal_report.joint_pitch,
                                   _gimbal_report.joint_yaw);
    float tilt;
    Vector3f velocity, euler, gyroBias;
    _ekf.getDebug(tilt, velocity, euler, gyroBias);
#if MOUNT_DEBUG
    ::printf("tilt=%.2f euler=(%.2f, %.2f, %.2f) yawRateDem=%.3f\n",
             tilt,
             degrees(euler.x), degrees(euler.y), degrees(euler.z),
             vehicleYawRateDem);
#endif
}

// provide a vehicle yaw rate demand in rad/sec
float AP_Mount_MAVLink::vehicleYawRateDemand(void)
{
    return vehicleYawRateDem;
}


#endif // AP_AHRS_NAVEKF_AVAILABLE
