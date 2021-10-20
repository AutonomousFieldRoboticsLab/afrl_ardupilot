#include <vector>
#include <numeric>

#include <AP_Math/AP_Math.h>

namespace utils
{
    void update_complementary_filter(Vector3f gyro_reading, Vector3f accel_reading, uint32_t last_update_millis, float accel_weight, float gyro_weight, float &fused_roll, float &fused_pitch)
    {
        float acceleration_norm = accel_reading.length();
        float time_delta_seconds = (static_cast<float>(last_update_millis) - static_cast<float>(AP_HAL::millis())) / 1000.0f;

        float ac_w = 0.0025;
        float accum_w = 1.0 - ac_w;

        fused_roll -= static_cast<float>(gyro_reading[0]) * time_delta_seconds;
        fused_pitch += static_cast<float>(gyro_reading[1]) * time_delta_seconds;

        // ensure we don't pack in a giant acceleration vector
        if (acceleration_norm < 12.0 && acceleration_norm > 6.5)
        {
            float pitch_accel = atan2f(accel_reading.x, accel_reading.z);
            float roll_accel = atan2f(accel_reading.y, accel_reading.z);

            fused_roll = atan2f(
                sinf(fused_roll) * accum_w + sinf(roll_accel) * ac_w,
                cosf(fused_roll) * accum_w + cosf(roll_accel) * ac_w);

            fused_pitch = atan2f(
                sinf(fused_pitch) * accum_w + sinf(pitch_accel) * ac_w,
                cosf(fused_pitch) * accum_w + cosf(pitch_accel) * ac_w);
        }
    }

    void add_performance_sample(std::vector<uint32_t> &samples, uint8_t sample_number, uint32_t sample_time)
    {
        samples[sample_number] = sample_time;
    }

    float get_sample_average(const std::vector<uint32_t> &samples)
    {
        float average = std::accumulate(samples.begin(), samples.end(), 0.0) / samples.size();
        return average / 1000.0;
    }
}