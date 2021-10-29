#pragma once

#include <AP_HAL/AP_HAL.h>

#include "include/mavlink/v2.0/ardupilotmega/version.h"
#include "include/mavlink/v2.0/mavlink_types.h"
#include "include/mavlink/v2.0/ardupilotmega/mavlink.h"

#include "Config.h"

class SimpleGCS
{
private:
    AP_HAL::UARTDriver *mavlink_uart;

    uint32_t last_heartbeat_message_time_;
    void send_heartbeat(void);

    MAV_TYPE frame_type() const { return MAV_TYPE_SUBMARINE; }
    MAV_MODE base_mode() const;
    uint32_t custom_mode() const;
    MAV_STATE vehicle_system_status() const;

    void send_mavlink_message(mavlink_message_t *msg);

public:
    SimpleGCS()
    {
        last_heartbeat_message_time_ = 0;
    };

    void init(AP_HAL::UARTDriver *uart);
    void update_receive();
    void handle_mavlink_message(const mavlink_message_t &msg);

    void send_text(MAV_SEVERITY severity, const char *format_string, ...);
    void send_imu_data(int16_t accel_x, int16_t accel_y, int16_t accel_z, int16_t gyro_x, int16_t gyro_y,
                       int16_t gyro_z, int16_t xmag, int16_t ymag, int16_t zmag, int16_t temperature);

    void send_attitude(float roll, float pitch, float yaw, float rollspeed, float pitchspeed, float yawspeed);
    void send_sensor_messages_if_needed(void);
    void send_heartbeat_if_needed(void);
    void send_scaled_pressure(float pressure, float temperature);
    void send_battery_status(float current_amps);
    void send_sys_status(float battery_voltage, float current_amps);

}; // class SimpleGCS
