// -*- tab-width: 4; Mode: C++; c-basic-offset: 4; indent-tabs-mode: nil -*-

/*
  MAVLink enabled mount backend class
 */

#ifndef __AP_MOUNT_MAVLINK_H__
#define __AP_MOUNT_MAVLINK_H__

#include <AP_HAL.h>
#include <AP_AHRS.h>

#if AP_AHRS_NAVEKF_AVAILABLE
#include <AP_Math.h>
#include <AP_Common.h>
#include <AP_GPS.h>
#include <GCS_MAVLink.h>
#include <RC_Channel.h>
#include <AP_Mount_Backend.h>
#include <AP_SmallEKF.h>

class AP_Mount_MAVLink : public AP_Mount_Backend
{

public:
    // Constructor
    AP_Mount_MAVLink(AP_Mount &frontend, AP_Mount::mount_state &state, uint8_t instance);

    // init - performs any required initialisation for this instance
    virtual void init(const AP_SerialManager& serial_manager);

    // update mount position - should be called periodically
    virtual void update();

    // has_pan_control - returns true if this mount can control it's pan (required for multicopters)
    virtual bool has_pan_control() const;

    // set_mode - sets mount's mode
    virtual void set_mode(enum MAV_MOUNT_MODE mode);

    // status_msg - called to allow mounts to send their status to GCS using the MOUNT_STATUS message
    virtual void status_msg(mavlink_channel_t chan);

    // handle a GIMBAL_REPORT message
    virtual void handle_gimbal_report(mavlink_channel_t chan, mavlink_message_t *msg);

    // send a GIMBAL_REPORT message to the GCS
    virtual void send_gimbal_report(mavlink_channel_t chan);

    // provide a vehicle yaw rate demand in rad/sec
    virtual float vehicleYawRateDemand(void);

private:
    // internal variables
    bool _initialised;              // true once the driver has been initialised

    // state of small EKF for gimbal
    SmallEKF _ekf;

    // keep last gimbal report
    mavlink_gimbal_report_t _gimbal_report;

    // vehicle yaw rate demand
    float vehicleYawRateDem;

    // gain from angle error to gimbal rate demand
    float const K_gimbalRate = 1.0f;

    // gain from angle error to vehicle yaw rate demand
    float const K_vehicleRate = 1.0f;

    // maximum gimbal angular rate in rad/sec
    float const angRateLimit = 0.5f;

    // maximum vehicle yaw rate in rad/sec
    float const vehYawRateLim = 1.0f;

    // gimbal yaw offset relative to vehicle reference frame in radians, used to centre relative to visual or mechanical limits
    float const gimbalYawOffset = 0.03f;

    // filtered yaw rate from the vehicle
    float vehicleYawRateFilt = 0.0f;

    // circular frequency (rad/sec) constant of filter applied to forward path vehicle yaw rate
    // this frequency must not be larger than the update rate (Hz).
    // reducing it makes the gimbal yaw less responsive to vehicle yaw
    // increasing it makes the gimbal yawe more responsive to vehicle yaw
    float const yawRateFiltPole = 10.0f;

    // amount of yaw angle that we permit the gimbal to lag the vehicle when operating in slave mode
    // reducing this makes the gimbal respond more to vehicle yaw disturbances
    float const yawErrorLimit = 0.1f;

};
#endif // AP_AHRS_NAVEKF_AVAILABLE

#endif // __AP_MOUNT_MAVLINK_H__
