#pragma once

namespace utils
{
    void update_complementary_filter(Vector3f gyro_reading, Vector3f accel_reading, uint32_t last_update_millis, float accel_weight, float gyro_weight, float &fused_roll, float &fused_pitch);
}