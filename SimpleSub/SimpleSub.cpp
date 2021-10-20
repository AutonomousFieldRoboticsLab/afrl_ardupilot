#include <cmath>

#include <AP_BoardConfig/AP_BoardConfig.h>
#include <AP_InertialSensor/AP_InertialSensor.h>

#include "Config.h"
#include "SimpleGCS.h"
#include "Sub.h"
#include "utils.h"

#if HAL_OS_POSIX_IO
#include <stdio.h>
#endif

void SimpleSub::send_imu_data_if_needed()
{
    // accel units from sensor are m/s/s
    // gyro units are rads/sec

    // scaled units are:
    //  gyro: mrads/sec
    // accel: mGs (weird unit)

    uint32_t current_time = AP_HAL::millis();

    // hal.console->printf("%ld \t %ld\r\n", current_time, last_imu_message_send_time_);
    if (current_time - last_imu_message_send_time_ > IMU_MESSAGE_RATE_MILLIS)
    {

        Vector3f acceleration_vector;
        Vector3f gyro_vector;

        acceleration_vector = inertial_sensor.get_accel(REPORT_IMU_INDEX);
        gyro_vector = inertial_sensor.get_gyro(REPORT_IMU_INDEX);

        utils::update_complementary_filter(
            gyro_vector,
            acceleration_vector,
            last_imu_message_send_time_,
            0.02,
            0.98,
            roll_filtered,
            pitch_filtered);

        float yaw = 0.0;
        float temperature = 0.0;

        gcs().send_imu_data(
            acceleration_vector.x * 1000.0 * 0.101972,
            acceleration_vector.y * 1000.0 * 0.101972,
            acceleration_vector.z * 1000.0 * 0.101972,
            gyro_vector.x * 1000.0,
            gyro_vector.y * 1000.0,
            gyro_vector.z * 1000.0,
            temperature);

        gcs().send_attitude(
            roll_filtered,
            pitch_filtered,
            yaw,
            gyro_vector.x,
            gyro_vector.y,
            gyro_vector.z);

        last_imu_message_send_time_ = current_time;
    }
}

void SimpleSub::send_pressure_if_needed()
{
    // accel units from sensor are m/s/s
    // gyro units are rads/sec

    // scaled units are:
    //  gyro: mrads/sec
    // accel: mGs (weird unit)

    uint32_t current_time = AP_HAL::millis();

    // hal.console->printf("%ld \t %ld\r\n", current_time, last_imu_message_send_time_);
    if (current_time - last_pressure_sent_time_ > PRESSURE_MESSAGE_RATE_MILLIS)
    {

        depth_sensor->update();
        float pressure = depth_sensor->get_pressure() * 0.01f;
        float temperature = depth_sensor->get_temprature() * 100;

        gcs().send_scaled_pressure(pressure, temperature);
        last_pressure_sent_time_ = current_time;
    }
}

bool SimpleSub::pwm_is_valid(uint16_t pwm)
{
    return pwm <= MAX_MOTOR_PWM && pwm >= MIN_MOTOR_PWM;
}

bool SimpleSub::set_motor_speed(uint16_t motor_index, uint16_t pwm)
{
    // gcs->send_text(MAV_SEVERITY_INFO, "Setting pwm %u on chan %u valid %d", pwm, motor_index, pwm_is_valid(pwm));
    if (motor_index >= NUMBER_MOTORS)
    {
        return false;
    }

    if (armed && pwm_is_valid(pwm))
    {
        current_motor_pwms[motor_index] = pwm;
        return true;
    }

    return false;
}

bool SimpleSub::any_motor_is_on()
{
    for (uint8_t motor_index = 0; motor_index < NUMBER_MOTORS; ++motor_index)
    {
        if (current_motor_pwms[motor_index] != NEUTRAL_MOTOR_PWM)
        {
            return true;
        }
    }

    return false;
}

void SimpleSub::set_speeds_to_stopped()
{
    for (int motor_index = 0; motor_index < NUMBER_MOTORS; ++motor_index)
    {
        current_motor_pwms[motor_index] = NEUTRAL_MOTOR_PWM;
    }
}

void SimpleSub::enable_motor_rc_channels()
{
    for (int motor_channel = 0; motor_channel < NUMBER_MOTORS; ++motor_channel)
    {
        rcout->enable_ch(motor_channel);
    }

    rcout->enable_ch(LIGHT_CHANNEL);

    rcout->set_freq(0xFF, MOTOR_PWM_FREQUENCY);

    rcout->force_safety_off();
}

void SimpleSub::output_to_motors()
{
    for (int motor_channel = 0; motor_channel < NUMBER_MOTORS; ++motor_channel)
    {
        // gcs().send_text(MAV_SEVERITY_NOTICE, "Motor %d: pwm %d", motor_channel, current_motor_pwms[motor_channel]);
        rcout->write(motor_channel, current_motor_pwms[motor_channel]);
    }

    // gcs().send_text(MAV_SEVERITY_WARNING, "light intensity: %d", light_intensity);
    rcout->write(LIGHT_CHANNEL, light_intensity);
}

MAV_RESULT SimpleSub::handle_command_long_packet(mavlink_command_long_t &command_long_packet)
{
    MAV_RESULT result = MAV_RESULT_FAILED;

    switch (command_long_packet.command)
    {
    case MAV_CMD_COMPONENT_ARM_DISARM:
        result = handle_command_component_arm_disarm(command_long_packet);
        break;

    default:
        result = MAV_RESULT_UNSUPPORTED;
        break;
    }

    return result;
}

MAV_RESULT SimpleSub::handle_command_component_arm_disarm(const mavlink_command_long_t &packet)
{
    if (is_equal(packet.param1, 1.0f))
    {
        gcs().send_text(MAV_SEVERITY_INFO, "Arming motors");
        arm();
        return MAV_RESULT_ACCEPTED;
    }

    if (is_zero(packet.param1))
    {
        gcs().send_text(MAV_SEVERITY_INFO, "Disarming motors");
        if (is_armed())
            disarm();
        return MAV_RESULT_ACCEPTED;
    }

    return MAV_RESULT_UNSUPPORTED;
}

bool SimpleSub::handle_rc_override_packet(mavlink_rc_channels_override_t rc_override_packet)
{
    bool success = set_motor_speed(0, rc_override_packet.chan1_raw) &&
                   set_motor_speed(1, rc_override_packet.chan2_raw) &&
                   set_motor_speed(2, rc_override_packet.chan3_raw) &&
                   set_motor_speed(3, rc_override_packet.chan4_raw) &&
                   set_motor_speed(4, rc_override_packet.chan5_raw) &&
                   set_motor_speed(5, rc_override_packet.chan6_raw);

    set_light_intensity(rc_override_packet.chan7_raw);
    if (success)
    {
        last_motor_control_message_time = AP_HAL::millis();
    }

    return success;
}

void SimpleSub::set_light_intensity(uint16_t light_intensity_)
{

    // gcs().send_text(MAV_SEVERITY_INFO, "Recieve chan 7: %d", light_intensity_);
    if (light_intensity_ == 1000)
    {
        light_intensity -= LIGHT_STEP;
    }
    else if (light_intensity_ == 2000)
    {
        light_intensity += LIGHT_STEP;
    }

    light_intensity = MAX(1000, MIN(light_intensity, 2000));
}

void SimpleSub::arm()
{
    armed = true;
}

void SimpleSub::disarm()
{
    set_speeds_to_stopped();
    output_to_motors();
    armed = false;
}

void SimpleSub::stop_if_delay_between_messages_too_long()
{
    uint32_t current_time = AP_HAL::millis();

    if (current_time < last_motor_control_message_time)
    {
        last_motor_control_message_time = AP_HAL::millis();
        gcs().send_text(MAV_SEVERITY_WARNING, "Stopping motors! Clock overflow!");
        set_speeds_to_stopped();
        return;
    }

    uint32_t time_gap = current_time - last_motor_control_message_time;

    if (time_gap > MAX_MOTOR_MESSAGE_RECEIVED_GAP_MILLIS)
    {
        if (any_motor_is_on())
        {
            last_motor_control_message_time = AP_HAL::millis();
            gcs().send_text(MAV_SEVERITY_WARNING, "Stopping motors! Too long since motor message!");
        }
        set_speeds_to_stopped();
    }

    return;
}

bool SimpleSub::is_armed()
{
    return armed;
}

AP_HAL_MAIN_CALLBACKS(&simple_sub);
