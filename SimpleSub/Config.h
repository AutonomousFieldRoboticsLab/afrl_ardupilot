#pragma once

const bool REPORT_PERFORMANCE_STATS = true;
const uint32_t PERFORMANCE_STATS_REPORT_RATE = 120000;

const bool SEND_SENSOR_MESSAGES = true;
// how often sensor stats are sent (SCALED_IMU only right now)
const uint32_t IMU_MESSAGE_RATE_MILLIS = 1;
const uint32_t PRESSURE_MESSAGE_RATE_MILLIS = 100;

const uint8_t REPORT_IMU_INDEX = 0;

// sets the rate at which the imu is polled. Keep this at zero or you will be sorry
const uint32_t AP_INERTIAL_SENSOR_IMU_SAMPLE_RATE_MILLIS = 0;

const uint32_t SERIAL_BAUD_RATE = 115200;
const uint32_t MAX_MOTOR_MESSAGE_RECEIVED_GAP_MILLIS = 2000;

const uint8_t SYSTEM_ID = 2;
const uint8_t COMPONENT_ID = 2;

const uint32_t HEART_BEAT_RATE_MILLIS = 1000;

const uint8_t PERFORMANCE_HISTORY_LENGTH = 20;

const uint8_t NUMBER_MOTORS = 6; // 0 - 5
const uint8_t LIGHT_CHANNEL = 6; // channel for light
const uint16_t LIGHT_STEP = 200;

const uint16_t MAX_MOTOR_PWM = 1900;
const uint16_t MIN_MOTOR_PWM = 1100;
const uint16_t NEUTRAL_MOTOR_PWM = 1500;
const uint8_t MOTOR_RC_CHANNEL_OFFSET = 0;

const uint16_t MOTOR_PWM_FREQUENCY = 490;
