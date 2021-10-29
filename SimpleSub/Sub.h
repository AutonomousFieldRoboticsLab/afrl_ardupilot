#pragma once

#include <vector>

#include <AP_BoardConfig/AP_BoardConfig.h>
#include <AP_BattMonitor/AP_BattMonitor.h>
#include <AP_InertialSensor/AP_InertialSensor.h>
#include <AP_SerialManager/AP_SerialManager.h>

#include "SimpleGCS.h"
#include "DepthSensor.h"
#include "Magnetometer.h"

// #define SIMPLE_SUB_DEBUG

// for filling in battery monitor initialization. From ArduSub
#define MASK_LOG_CURRENT (1 << 9)

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

    // avoid using gcs directly, use get_gcs() instead
    SimpleGCS _gcs;
    SimpleGCS &gcs() { return _gcs; }

    // board specific config
    AP_BoardConfig BoardConfig;
    // Sensors Drivers
    AP_InertialSensor inertial_sensor;
    AP_SerialManager serial_manager;
    DepthSensor *depth_sensor;
    Magnetometer *magnetometer;
    AP_HAL::RCOutput *rcout;
    // Battery Sensors
    AP_BattMonitor battery{MASK_LOG_CURRENT, FUNCTOR_BIND_MEMBER(&SimpleSub::handle_battery_failsafe, void, const char *, const int8_t),
                           _failsafe_priorities};

    float get_battery_voltage();
    float get_current_consumption_amps();

private:
    bool armed;

    float roll_filtered;
    float pitch_filtered;

    uint8_t num_inertial_sensors_;

    uint32_t last_motor_control_message_time;
    uint32_t last_imu_message_send_time_;
    uint32_t last_pressure_sent_time_;
    uint32_t last_battery_status_message_time_;
    std::vector<uint16_t> current_motor_pwms;

    static SimpleSub *_singleton;

    bool battery_failsafe_triggered;

    void enable_motor_rc_channels();
    void output_to_motors();
    bool pwm_is_valid(uint16_t pwm);
    bool set_motor_speed(uint16_t motor_index, uint16_t pwm);
    bool any_motor_is_on();
    void set_speeds_to_stopped();
    void stop_if_delay_between_messages_too_long();

    uint16_t light_intensity;
    void set_light_intensity(uint16_t light_intensity_);

    void send_imu_data_if_needed();
    void send_pressure_if_needed();
    void send_battery_status_if_needed();
    void report_performance_stats_if_needed();

    MAV_RESULT handle_command_long_packet(mavlink_command_long_t &command_long_packet);
    MAV_RESULT handle_command_component_arm_disarm(const mavlink_command_long_t &packet);
    bool handle_rc_override_packet(mavlink_rc_channels_override_t rc_override_packet);

    // Simply logs out the failsafe type
    void handle_battery_failsafe(const char *type, const int8_t action);
    void update_battery();

    void arm();
    void disarm();
    bool is_armed();

    uint32_t last_main_loop_time_;
    uint32_t last_performance_report_time_;
    std::vector<uint32_t> main_loop_rate_samples;
    std::vector<uint32_t> motor_control_packet_rate_samples;

    enum Failsafe_Action
    {
        Failsafe_Action_None = 0,
        Failsafe_Action_Warn = 1,
        Failsafe_Action_Disarm = 2,
        Failsafe_Action_Surface = 3
    };

    // taken from ArduSub
    static constexpr int8_t _failsafe_priorities[] = {
        Failsafe_Action_Disarm,
        Failsafe_Action_Surface,
        Failsafe_Action_Warn,
        Failsafe_Action_None,
        -1 // the priority list must end with a sentinel of -1
    };
}; // class SimpleSub

extern const AP_HAL::HAL &hal;
extern SimpleSub simple_sub;
