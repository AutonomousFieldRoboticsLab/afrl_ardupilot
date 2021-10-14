#include <AP_Math/AP_Math.h>

#include "Sub.h"

void SimpleSub::setup(void)
{
    serial_manager.init_console();

    BoardConfig.init();

    simple_sub.disarm();

    gcs().init(hal.uartA);

    inertial_sensor.init(AP_INERTIAL_SENSOR_IMU_SAMPLE_RATE_MILLIS);
    inertial_sensor.update();

    uint8_t accel_count = inertial_sensor.get_accel_count();
    uint8_t gyro_count = inertial_sensor.get_gyro_count();
    num_inertial_sensors_ = MAX(accel_count, gyro_count);

    rcout = hal.rcout;
    current_motor_pwms.resize(NUMBER_MOTORS);
    hal.scheduler->delay(100);
    enable_motor_rc_channels();
    set_speeds_to_stopped();

    light_intensity = 1000;
}

void SimpleSub::loop(void)
{

    gcs().update_receive();

    inertial_sensor.wait_for_sample();
    inertial_sensor.update();

    // highest priority is sending the motors
    simple_sub.stop_if_delay_between_messages_too_long();
    simple_sub.output_to_motors();

    gcs().send_heartbeat_if_needed();

    // if (REPORT_PERFORMANCE_STATS)
    // {
    //     gcs().report_performance_stats_if_needed();
    // }

    send_sensor_messages_if_needed();

    // add_performance_sample(main_loop_rate_samples, main_loop_number, last_main_loop_time);
}