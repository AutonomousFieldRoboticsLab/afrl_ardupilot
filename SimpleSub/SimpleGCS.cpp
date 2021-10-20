#include "AP_HAL/AP_HAL.h"

#include "SimpleGCS.h"
#include "Sub.h"

void SimpleGCS::init(AP_HAL::UARTDriver *uart)
{
    mavlink_uart = uart;
    mavlink_uart->begin(SERIAL_BAUD_RATE);
}

uint32_t SimpleGCS::custom_mode() const
{
    return simple_sub.is_armed() ? MAV_MODE_MANUAL_ARMED : MAV_MODE_MANUAL_DISARMED;
}

MAV_MODE SimpleGCS::base_mode() const
{
    return simple_sub.is_armed() ? MAV_MODE_MANUAL_ARMED : MAV_MODE_MANUAL_DISARMED;
}

MAV_STATE SimpleGCS::vehicle_system_status() const
{
    if (simple_sub.is_armed())
        return MAV_STATE_ACTIVE;

    return MAV_STATE_STANDBY;
}

void SimpleGCS::update_receive()
{
    // receive new packets
    mavlink_message_t msg;
    mavlink_status_t status;
    const uint16_t nbytes = mavlink_uart->available();

    for (uint16_t i = 0; i < nbytes; i++)
    {
        const uint8_t c = (uint8_t)mavlink_uart->read();

#ifdef SIMPLE_SUB_DEBUG
        // send_text(MAV_SEVERITY_INFO, "Got byte %c", received_byte);
#endif
        if (mavlink_parse_char(MAVLINK_COMM_0, c, &msg, &status))
        {
            handle_mavlink_message(msg);
        }
    }
}

void SimpleGCS::handle_mavlink_message(const mavlink_message_t &msg)
{
    switch (msg.msgid)
    {
    case MAVLINK_MSG_ID_COMMAND_LONG:
    {
        mavlink_command_long_t command_long_packet;
        mavlink_msg_command_long_decode(&msg, &command_long_packet);

        const MAV_RESULT result = simple_sub.handle_command_long_packet(command_long_packet);

        mavlink_message_t ack_msg;
        mavlink_msg_command_ack_pack(SYSTEM_ID, COMPONENT_ID, &ack_msg, command_long_packet.command, result);
        send_mavlink_message(&ack_msg);

#ifdef SIMPLE_SUB_DEBUG
        send_text(MAV_SEVERITY_INFO, "Got command long packet with command %u", command_long_packet.command);
#endif
        break;
    }

    case MAVLINK_MSG_ID_RC_CHANNELS_OVERRIDE:
    {
        // reformats as a direct motor control packet runs
        mavlink_rc_channels_override_t rc_override_message;
        mavlink_msg_rc_channels_override_decode(&msg, &rc_override_message);
        // send_text(MAV_SEVERITY_INFO, "Got rc override packet");
        simple_sub.handle_rc_override_packet(rc_override_message);
        break;
    }

    case MAVLINK_MSG_ID_REQUEST_DATA_STREAM:
    {
#ifdef SIMPLE_SUB_DEBUG
        send_text(MAV_SEVERITY_INFO, "Request data stream ignored.");
#endif
        break;
    }
    case MAVLINK_MSG_ID_HEARTBEAT:
    {
#ifdef SIMPLE_SUB_DEBUG
        send_text(MAV_SEVERITY_INFO, "Heartbeat ignored.");
#endif
        break;
    }
    case MAVLINK_MSG_ID_PARAM_REQUEST_LIST:
    {
#ifdef SIMPLE_SUB_DEBUG
        send_text(MAV_SEVERITY_INFO, "Param request list ignored.");
#endif
        break;
    }
    default:
    {
#ifdef SIMPLE_SUB_DEBUG
        send_text(MAV_SEVERITY_INFO, "Unrecognized message id! %u", msg.msgid);
#endif
        break;
    }
    }
}

void SimpleGCS::send_text(MAV_SEVERITY severity, const char *format_string, ...)
{
    // formats and sends a debug text
    va_list format_args;
    va_start(format_args, format_string);

    mavlink_message_t message;

    char trimmed_string[MAVLINK_MSG_STATUSTEXT_FIELD_TEXT_LEN];
    int number_bytes_written = vsnprintf(trimmed_string, MAVLINK_MSG_STATUSTEXT_FIELD_TEXT_LEN, format_string, format_args);
    va_end(format_args);

    if (number_bytes_written < 0 || number_bytes_written > MAVLINK_MSG_STATUSTEXT_FIELD_TEXT_LEN)
    {
    }

    for (int i = number_bytes_written; i < MAVLINK_MSG_STATUSTEXT_FIELD_TEXT_LEN; ++i)
    {
        trimmed_string[i] = ' ';
    }

    mavlink_msg_statustext_pack(SYSTEM_ID, COMPONENT_ID, &message, severity, trimmed_string, 0, 0);
    send_mavlink_message(&message);
}

void SimpleGCS::send_imu_data(int16_t accel_x, int16_t accel_y, int16_t accel_z, int16_t gyro_x, int16_t gyro_y, int16_t gyro_z, int16_t temperature)
{

    mavlink_message_t msg;
    mavlink_msg_scaled_imu_pack(
        SYSTEM_ID,
        COMPONENT_ID,
        &msg,
        AP_HAL::millis(),
        accel_x,
        accel_y,
        accel_z,
        gyro_x,
        gyro_y,
        gyro_z,
        0,
        0,
        0,
        temperature);

    send_mavlink_message(&msg);
}

void SimpleGCS::send_attitude(float roll, float pitch, float yaw, float rollspeed, float pitchspeed, float yawspeed)
{
    mavlink_message_t msg;
    mavlink_msg_attitude_pack(
        SYSTEM_ID,
        COMPONENT_ID,
        &msg,
        AP_HAL::millis(),
        roll,
        pitch,
        yaw,
        rollspeed,
        pitchspeed,
        yawspeed);

    send_mavlink_message(&msg);
}

void SimpleGCS::send_heartbeat_if_needed(void)
{
    uint32_t current_time = AP_HAL::millis();
    uint32_t time_gap = current_time - last_heartbeat_message_time_;
    if (HEART_BEAT_RATE_MILLIS < time_gap)
    {
#ifdef SIMPLE_SUB_DEBUG
        send_text(MAV_SEVERITY_INFO, "Sending heart beat message");
#endif
        send_heartbeat();
    }
    last_heartbeat_message_time_ = current_time;
}

void SimpleGCS::send_heartbeat()
{

    mavlink_message_t msg;
    mavlink_msg_heartbeat_pack(
        SYSTEM_ID,
        COMPONENT_ID,
        &msg,
        frame_type(),
        MAV_AUTOPILOT_ARDUPILOTMEGA,
        base_mode(),
        0,
        vehicle_system_status());

    send_mavlink_message(&msg);
}

void SimpleGCS::send_scaled_pressure(float pressure, float temperature)
{
    mavlink_message_t msg;
    mavlink_msg_scaled_pressure_pack(
        SYSTEM_ID,
        COMPONENT_ID,
        &msg,
        AP_HAL::millis(),
        pressure,
        0.0,
        temperature);

    send_mavlink_message(&msg);
}

void SimpleGCS::send_mavlink_message(mavlink_message_t *msg)
{

    uint8_t message_buffer[MAVLINK_MAX_PACKET_LEN];
    size_t message_buffer_length = mavlink_msg_to_send_buffer(message_buffer, msg);
    mavlink_uart->write(message_buffer, message_buffer_length);
}
