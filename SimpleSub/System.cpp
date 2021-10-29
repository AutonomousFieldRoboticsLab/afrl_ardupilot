#include <AP_Math/AP_Math.h>

#include "Sub.h"
#include "utils.h"

void SimpleSub::setup(void)
{
    serial_manager.init_console();

    BoardConfig.init();

    gcs().init(hal.uartA);

    inertial_sensor.init(AP_INERTIAL_SENSOR_IMU_SAMPLE_RATE_MILLIS);
    inertial_sensor.update();

    uint8_t accel_count = inertial_sensor.get_accel_count();
    uint8_t gyro_count = inertial_sensor.get_gyro_count();
    num_inertial_sensors_ = MAX(accel_count, gyro_count);

    rcout = hal.rcout;
    current_motor_pwms.resize(NUMBER_MOTORS);
    hal.scheduler->delay(10);
    enable_motor_rc_channels();
    simple_sub.disarm();

    light_intensity = 1000;

    depth_sensor = DepthSensor::probe(std::move(GET_I2C_DEVICE(1, HAL_BARO_MS5837_I2C_ADDR)), DepthSensor::BARO_MS5837);
    if (!depth_sensor)
    {
        gcs().send_text(MAV_SEVERITY_CRITICAL, "Depth Sensor nullptr");
        hal.scheduler->delay(10);
    }

    depth_sensor->update();

    magnetometer = Magnetometer::probe_mpu9250(0, ROTATION_ROLL_180_YAW_90);
    if (!magnetometer)
    {
        gcs().send_text(MAV_SEVERITY_CRITICAL, "magnetometer nullptr");
        hal.scheduler->delay(100);
    }
    // For performance reporting
    main_loop_rate_samples.resize(PERFORMANCE_HISTORY_LENGTH);
    motor_control_packet_rate_samples.resize(PERFORMANCE_HISTORY_LENGTH);

    serial_manager.set_blocking_writes_all(false);

    // a bit of a nasty hack. Used to go around the ardupilot AP_Param system
    simple_sub.battery._params[0]._type.set(AP_BattMonitor_Params::BattMonitor_Type::BattMonitor_TYPE_ANALOG_VOLTAGE_AND_CURRENT);
    simple_sub.battery._params[0]._volt_multiplier.set(VOLTAGE_SENSOR_VOLTAGE_MULTIPLIER);
    simple_sub.battery._params[0]._curr_amp_per_volt.set(CURRENT_SENSOR_AMPS_PER_VOLT);
    simple_sub.battery.init();
    battery_failsafe_triggered = false;

    last_main_loop_time_ = AP_HAL::millis();
}

static uint8_t loop_count = 0;

void SimpleSub::loop(void)
{

    gcs().update_receive();

    inertial_sensor.wait_for_sample();
    inertial_sensor.update();

    // highest priority is sending the motors
    simple_sub.stop_if_delay_between_messages_too_long();
    simple_sub.output_to_motors();

    gcs().send_heartbeat_if_needed();

    if (REPORT_PERFORMANCE_STATS)
    {
        report_performance_stats_if_needed();
    }

    send_imu_data_if_needed();
    send_pressure_if_needed();
    send_battery_status_if_needed();

    uint32_t current_time = AP_HAL::millis();
    uint32_t span = current_time - last_main_loop_time_;
    utils::add_performance_sample(main_loop_rate_samples, loop_count, span);
    loop_count++;
    loop_count %= PERFORMANCE_HISTORY_LENGTH;
    last_main_loop_time_ = current_time;
}

void SimpleSub::report_performance_stats_if_needed()
{
    // rate of main loop
    // rate of motor packets being received

    if (AP_HAL::millis() - last_performance_report_time_ > PERFORMANCE_STATS_REPORT_RATE)
    {
        float main_rate = utils::get_sample_average(main_loop_rate_samples);
        // float motor_rate = utils::get_sample_average(motor_control_packet_rate_samples);

        gcs().send_text(MAV_SEVERITY_INFO, "Main rate: %f", main_rate);

        last_performance_report_time_ = AP_HAL::millis();
    }
}