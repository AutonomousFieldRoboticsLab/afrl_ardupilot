/**
 * @file DepthSensor.h
 * @author  Bharat Joshi (bjoshi@email.sc.edu)
 * @brief This is an extension of AP_Baro_MS56XX.cpp to be used as standalone barometer driver. Using
 *  AP_Baro interface seems too tricky at best
 * @version 0.1
 * @date 2021-10-19
 *
 * @copyright Copyright (c) 2021
 *
 */

#pragma once

#include <AP_HAL/AP_HAL.h>
#include <AP_HAL/Semaphores.h>
#include <AP_HAL/Device.h>
#include <AP_HAL/I2CDevice.h>

#ifndef HAL_BARO_MS5611_I2C_ADDR
#define HAL_BARO_MS5611_I2C_ADDR 0x77
#endif

#ifndef HAL_BARO_MS5611_I2C_ADDR2
#define HAL_BARO_MS5611_I2C_ADDR2 0x76
#endif

#ifndef HAL_BARO_MS5607_I2C_ADDR
#define HAL_BARO_MS5607_I2C_ADDR 0x77
#endif

#ifndef HAL_BARO_MS5837_I2C_ADDR
#define HAL_BARO_MS5837_I2C_ADDR 0x76
#endif

#ifndef HAL_BARO_MS5637_I2C_ADDR
#define HAL_BARO_MS5637_I2C_ADDR 0x76
#endif

// timeouts for health reporting
#define BARO_TIMEOUT_MS 500              // timeout in ms since last successful read
#define BARO_DATA_CHANGE_TIMEOUT_MS 2000 // timeout in ms since last successful read that involved temperature of pressure changing

#define GET_I2C_DEVICE(bus, address) hal.i2c_mgr->get_device(bus, address)

class DepthSensor
{
public:
    void update();
    void calibrate(bool save);

    enum MS56XX_TYPE
    {
        BARO_MS5611 = 0,
        BARO_MS5607 = 1,
        BARO_MS5637 = 2,
        BARO_MS5837 = 3
    };

    // barometer types
    typedef enum
    {
        BARO_TYPE_AIR,
        BARO_TYPE_WATER
    } baro_type_t;

    static DepthSensor *probe(AP_HAL::OwnPtr<AP_HAL::Device> dev, enum MS56XX_TYPE ms56xx_type = BARO_MS5611);

    // pressure in Pascal. Divide by 100 for millibars or hectopascals
    float get_pressure(void) const;

    // temperature in degrees C
    float get_temprature(void) const;

    // void calibrate();

private:
    /*
     * Update @accum and @count with the new sample in @val, taking into
     * account a maximum number of samples given by @max_count; in case
     * maximum number is reached, @accum and @count are updated appropriately
     */
    static void _update_and_wrap_accumulator(uint32_t *accum, uint32_t val,
                                             uint8_t *count, uint8_t max_count);

    DepthSensor(AP_HAL::OwnPtr<AP_HAL::Device> dev, enum MS56XX_TYPE ms56xx_type);

    bool _init();

    void _calculate_5611();
    void _calculate_5607();
    void _calculate_5637();
    void _calculate_5837();
    bool _read_prom_5611(uint16_t prom[8]);
    bool _read_prom_5637(uint16_t prom[8]);

    uint16_t _read_prom_word(uint8_t word);
    uint32_t _read_adc();

    void _timer();

    AP_HAL::OwnPtr<AP_HAL::Device> _dev;

    /* Shared values between thread sampling the HW and main thread */
    struct
    {
        uint32_t s_D1;
        uint32_t s_D2;
        uint8_t d1_count;
        uint8_t d2_count;
    } _accum;

    uint8_t _state;

    /* Last compensated values from accumulated sample */
    float _D1, _D2;

    // Internal calibration registers
    struct
    {
        uint16_t c1, c2, c3, c4, c5, c6;
    } _cal_reg;

    bool _discard_next;

    enum MS56XX_TYPE _ms56xx_type;

    // AP_Float _alt_offset;

    float filter_range;
    // mean pressure for range filter
    float _mean_pressure;
    // number of dropped samples. Not used for now, but can be usable to choose more reliable sensor
    uint32_t _error_count;

    void update_sensor_status(float pressure, float temprature);
    void update_healthy_flag(void);

    bool pressure_ok(float);

    struct sensor_status
    {
        uint32_t last_update_ms; // last update time in ms
        uint32_t last_change_ms; // last update time in ms that included a change in reading from previous readings
        float pressure;          // pressure in Pascal
        float temperature;       // temperature in degrees C
        float altitude;          // calculated altitude
        // AP_Float ground_pressure;
        float p_correction;
        baro_type_t type; // 0 for air pressure (default), 1 for water pressure
        bool healthy;     // true if sensor is healthy
        bool alt_ok;      // true if calculated altitude is ok
        bool calibrated;  // true if calculated calibrated successfully
    };
    sensor_status depth_sensor_status;
};
