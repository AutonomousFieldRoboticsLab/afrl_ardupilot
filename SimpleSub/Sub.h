#pragma once

#include <vector>

#include <AP_BoardConfig/AP_BoardConfig.h>
#include <AP_InertialSensor/AP_InertialSensor.h>
#include <AP_SerialManager/AP_SerialManager.h>

#include "SimpleGCS.h"

// #define SIMPLE_SUB_DEBUG

class SimpleSub : public AP_HAL::HAL::Callbacks
{

public:
    friend class SimpleGCS;

    SimpleSub();
    ~SimpleSub() = default;
    /* Do not allow copies */
    SimpleSub(const SimpleSub &other) = delete;
    SimpleSub &operator=(const SimpleSub &) = delete;

    static SimpleSub *get_singleton();

    // setup is called once at startup
    // loop is called continously - no delay
    void setup() override final;
    void loop() override final;

    //avoid using gcs directly, use get_gcs() instead
    SimpleGCS _gcs;
    SimpleGCS &gcs() { return _gcs; }

    // board specific config
    AP_BoardConfig BoardConfig;
    // Sensors Drivers
    AP_InertialSensor inertial_sensor;
    AP_SerialManager serial_manager;

    AP_HAL::RCOutput *rcout;

private:
    bool armed;

    float roll_filtered;
    float pitch_filtered;

    uint8_t num_inertial_sensors_;
    uint32_t last_motor_control_message_time;
    uint32_t last_imu_message_send_time_;

    std::vector<uint16_t> current_motor_pwms;

    static SimpleSub *_singleton;

    bool pwm_is_valid(uint16_t pwm);
    bool set_motor_speed(uint16_t motor_index, uint16_t pwm);
    bool any_motor_is_on();
    void set_speeds_to_stopped();
    void enable_motor_rc_channels();
    void output_to_motors();
    void stop_if_delay_between_messages_too_long();

    void send_sensor_messages_if_needed();

    MAV_RESULT handle_command_long_packet(mavlink_command_long_t &command_long_packet);
    MAV_RESULT handle_command_component_arm_disarm(const mavlink_command_long_t &packet);
    bool handle_rc_override_packet(mavlink_rc_channels_override_t rc_override_packet);

    void arm();
    void disarm();
    bool is_armed();

}; // class SimpleSub

extern const AP_HAL::HAL &hal;
extern SimpleSub simple_sub;
