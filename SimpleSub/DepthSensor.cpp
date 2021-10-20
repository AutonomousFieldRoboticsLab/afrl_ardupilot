/*
   This program is free software: you can redistribute it and/or modify
   it under the terms of the GNU General Public License as published by
   the Free Software Foundation, either version 3 of the License, or
   (at your option) any later version.

   This program is distributed in the hope that it will be useful,
   but WITHOUT ANY WARRANTY; without even the implied warranty of
   MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
   GNU General Public License for more details.

   You should have received a copy of the GNU General Public License
   along with this program.  If not, see <http://www.gnu.org/licenses/>.
 */
#include "DepthSensor.h"

#include <utility>
#include <stdio.h>

#include <AP_Math/AP_Math.h>
#include <AP_Math/crc.h>

#include "Sub.h"

extern const AP_HAL::HAL &hal;

static const uint8_t CMD_MS56XX_RESET = 0x1E;
static const uint8_t CMD_MS56XX_READ_ADC = 0x00;

/* PROM start address */
static const uint8_t CMD_MS56XX_PROM = 0xA0;

/* write to one of these addresses to start pressure conversion */
#define ADDR_CMD_CONVERT_D1_OSR256 0x40
#define ADDR_CMD_CONVERT_D1_OSR512 0x42
#define ADDR_CMD_CONVERT_D1_OSR1024 0x44
#define ADDR_CMD_CONVERT_D1_OSR2048 0x46
#define ADDR_CMD_CONVERT_D1_OSR4096 0x48

/* write to one of these addresses to start temperature conversion */
#define ADDR_CMD_CONVERT_D2_OSR256 0x50
#define ADDR_CMD_CONVERT_D2_OSR512 0x52
#define ADDR_CMD_CONVERT_D2_OSR1024 0x54
#define ADDR_CMD_CONVERT_D2_OSR2048 0x56
#define ADDR_CMD_CONVERT_D2_OSR4096 0x58

/*
  use an OSR of 1024 to reduce the self-heating effect of the
  sensor. Information from MS tells us that some individual sensors
  are quite sensitive to this effect and that reducing the OSR can
  make a big difference
 */
static const uint8_t ADDR_CMD_CONVERT_PRESSURE = ADDR_CMD_CONVERT_D1_OSR1024;
static const uint8_t ADDR_CMD_CONVERT_TEMPERATURE = ADDR_CMD_CONVERT_D2_OSR1024;

static constexpr float FILTER_KOEF = 0.1f;

/*
  constructor
 */
DepthSensor::DepthSensor(AP_HAL::OwnPtr<AP_HAL::Device> dev, enum MS56XX_TYPE ms56xx_type)
    : _dev(std::move(dev)), _ms56xx_type(ms56xx_type)
{
    filter_range = 13.0f;
}

DepthSensor *DepthSensor::probe(AP_HAL::OwnPtr<AP_HAL::Device> dev,
                                enum MS56XX_TYPE ms56xx_type)
{
    if (!dev)
    {
        return nullptr;
    }
    DepthSensor *sensor = new DepthSensor(std::move(dev), ms56xx_type);
    if (!sensor || !sensor->_init())
    {
        delete sensor;
        return nullptr;
    }
    return sensor;
}

bool DepthSensor::_init()
{
    if (!_dev)
    {
        return false;
    }

    _dev->get_semaphore()->take_blocking();

    // high retries for init
    _dev->set_retries(10);

    uint16_t prom[8];
    bool prom_read_ok = false;

    _dev->transfer(&CMD_MS56XX_RESET, 1, nullptr, 0);
    hal.scheduler->delay(4);

    const char *name = "MS5611";
    switch (_ms56xx_type)
    {
    case BARO_MS5607:
        name = "MS5607";
        FALLTHROUGH;
    case BARO_MS5611:
        prom_read_ok = _read_prom_5611(prom);
        break;
    case BARO_MS5837:
        name = "MS5837";
        prom_read_ok = _read_prom_5637(prom);
        break;
    case BARO_MS5637:
        name = "MS5637";
        prom_read_ok = _read_prom_5637(prom);
        break;
    }

    if (!prom_read_ok)
    {
        _dev->get_semaphore()->give();
        return false;
    }

    printf("%s found on bus %u address 0x%02x\n", name, _dev->bus_num(), _dev->get_bus_address());

    // Save factory calibration coefficients
    _cal_reg.c1 = prom[1];
    _cal_reg.c2 = prom[2];
    _cal_reg.c3 = prom[3];
    _cal_reg.c4 = prom[4];
    _cal_reg.c5 = prom[5];
    _cal_reg.c6 = prom[6];

    // Send a command to read temperature first
    _dev->transfer(&ADDR_CMD_CONVERT_TEMPERATURE, 1, nullptr, 0);
    _state = 0;

    memset(&_accum, 0, sizeof(_accum));

    if (_ms56xx_type == BARO_MS5837)
    {
        depth_sensor_status.type = BARO_TYPE_WATER;
    }

    // lower retries for run
    _dev->set_retries(3);

    _dev->get_semaphore()->give();

    /* Request 100Hz update */
    _dev->register_periodic_callback(10 * AP_USEC_PER_MSEC,
                                     FUNCTOR_BIND_MEMBER(&DepthSensor::_timer, void));
    return true;
}

uint16_t DepthSensor::_read_prom_word(uint8_t word)
{
    const uint8_t reg = CMD_MS56XX_PROM + (word << 1);
    uint8_t val[2];
    if (!_dev->transfer(&reg, 1, val, sizeof(val)))
    {
        return 0;
    }
    return (val[0] << 8) | val[1];
}

uint32_t DepthSensor::_read_adc()
{
    uint8_t val[3];
    if (!_dev->transfer(&CMD_MS56XX_READ_ADC, 1, val, sizeof(val)))
    {
        return 0;
    }
    return (val[0] << 16) | (val[1] << 8) | val[2];
}

bool DepthSensor::_read_prom_5611(uint16_t prom[8])
{
    /*
     * MS5611-01BA datasheet, CYCLIC REDUNDANCY CHECK (CRC): "MS5611-01BA
     * contains a PROM memory with 128-Bit. A 4-bit CRC has been implemented
     * to check the data validity in memory."
     *
     * CRC field must me removed for CRC-4 calculation.
     */
    bool all_zero = true;
    for (uint8_t i = 0; i < 8; i++)
    {
        prom[i] = _read_prom_word(i);
        if (prom[i] != 0)
        {
            all_zero = false;
        }
    }

    if (all_zero)
    {
        return false;
    }

    /* save the read crc */
    const uint16_t crc_read = prom[7] & 0xf;

    /* remove CRC byte */
    prom[7] &= 0xff00;

    return crc_read == crc_crc4(prom);
}

bool DepthSensor::_read_prom_5637(uint16_t prom[8])
{
    /*
     * MS5637-02BA03 datasheet, CYCLIC REDUNDANCY CHECK (CRC): "MS5637
     * contains a PROM memory with 112-Bit. A 4-bit CRC has been implemented
     * to check the data validity in memory."
     *
     * 8th PROM word must be zeroed and CRC field removed for CRC-4
     * calculation.
     */
    bool all_zero = true;
    for (uint8_t i = 0; i < 7; i++)
    {
        prom[i] = _read_prom_word(i);
        if (prom[i] != 0)
        {
            all_zero = false;
        }
    }

    if (all_zero)
    {
        return false;
    }

    prom[7] = 0;

    /* save the read crc */
    const uint16_t crc_read = (prom[0] & 0xf000) >> 12;

    /* remove CRC byte */
    prom[0] &= ~0xf000;

    return crc_read == crc_crc4(prom);
}

/*
 * Read the sensor with a state machine
 * We read one time temperature (state=0) and then 4 times pressure (states 1-4)
 *
 * Temperature is used to calculate the compensated pressure and doesn't vary
 * as fast as pressure. Hence we reuse the same temperature for 4 samples of
 * pressure.
 */
void DepthSensor::_timer(void)
{
    uint8_t next_cmd;
    uint8_t next_state;
    uint32_t adc_val = _read_adc();

    /*
     * If read fails, re-initiate a read command for current state or we are
     * stuck
     */
    if (adc_val == 0)
    {
        next_state = _state;
    }
    else
    {
        next_state = (_state + 1) % 5;
    }

    next_cmd = next_state == 0 ? ADDR_CMD_CONVERT_TEMPERATURE
                               : ADDR_CMD_CONVERT_PRESSURE;
    if (!_dev->transfer(&next_cmd, 1, nullptr, 0))
    {
        return;
    }

    /* if we had a failed read we are all done */
    if (adc_val == 0 || adc_val == 0xFFFFFF)
    {
        // a failed read can mean the next returned value will be
        // corrupt, we must discard it. This copes with MISO being
        // pulled either high or low
        _discard_next = true;
        return;
    }

    if (_discard_next)
    {
        _discard_next = false;
        _state = next_state;
        return;
    }

    if (_state == 0)
    {
        _update_and_wrap_accumulator(&_accum.s_D2, adc_val,
                                     &_accum.d2_count, 32);
    }
    else if (pressure_ok(adc_val))
    {
        _update_and_wrap_accumulator(&_accum.s_D1, adc_val,
                                     &_accum.d1_count, 128);
    }

    _state = next_state;
}

void DepthSensor::_update_and_wrap_accumulator(uint32_t *accum, uint32_t val,
                                               uint8_t *count, uint8_t max_count)
{
    *accum += val;
    *count += 1;
    if (*count == max_count)
    {
        *count = max_count / 2;
        *accum = *accum / 2;
    }
}

void DepthSensor::update()
{
    uint32_t sD1, sD2;
    uint8_t d1count, d2count;

    if (_accum.d1_count == 0)
    {
        return;
    }

    sD1 = _accum.s_D1;
    sD2 = _accum.s_D2;
    d1count = _accum.d1_count;
    d2count = _accum.d2_count;
    memset(&_accum, 0, sizeof(_accum));

    if (d1count != 0)
    {
        _D1 = ((float)sD1) / d1count;
    }
    if (d2count != 0)
    {
        _D2 = ((float)sD2) / d2count;
    }

    switch (_ms56xx_type)
    {
    case BARO_MS5607:
        _calculate_5607();
        break;
    case BARO_MS5611:
        _calculate_5611();
        break;
    case BARO_MS5637:
        _calculate_5637();
        break;
    case BARO_MS5837:
        _calculate_5837();
    }
}

// Calculate Temperature and compensated Pressure in real units (Celsius degrees*100, mbar*100).
void DepthSensor::_calculate_5611()
{
    float dT;
    float TEMP;
    float OFF;
    float SENS;

    // Formulas from manufacturer datasheet
    // sub -15c temperature compensation is not included

    // we do the calculations using floating point allows us to take advantage
    // of the averaging of D1 and D1 over multiple samples, giving us more
    // precision
    dT = _D2 - (((uint32_t)_cal_reg.c5) << 8);
    TEMP = (dT * _cal_reg.c6) / 8388608;
    OFF = _cal_reg.c2 * 65536.0f + (_cal_reg.c4 * dT) / 128;
    SENS = _cal_reg.c1 * 32768.0f + (_cal_reg.c3 * dT) / 256;

    if (TEMP < 0)
    {
        // second order temperature compensation when under 20 degrees C
        float T2 = (dT * dT) / 0x80000000;
        float Aux = TEMP * TEMP;
        float OFF2 = 2.5f * Aux;
        float SENS2 = 1.25f * Aux;
        TEMP = TEMP - T2;
        OFF = OFF - OFF2;
        SENS = SENS - SENS2;
    }

    float pressure = (_D1 * SENS / 2097152 - OFF) / 32768;
    float temperature = (TEMP + 2000) * 0.01f;
    update_sensor_status(pressure, temperature);
}

// Calculate Temperature and compensated Pressure in real units (Celsius degrees*100, mbar*100).
void DepthSensor::_calculate_5607()
{
    float dT;
    float TEMP;
    float OFF;
    float SENS;

    // Formulas from manufacturer datasheet
    // sub -15c temperature compensation is not included

    // we do the calculations using floating point allows us to take advantage
    // of the averaging of D1 and D1 over multiple samples, giving us more
    // precision
    dT = _D2 - (((uint32_t)_cal_reg.c5) << 8);
    TEMP = (dT * _cal_reg.c6) / 8388608;
    OFF = _cal_reg.c2 * 131072.0f + (_cal_reg.c4 * dT) / 64;
    SENS = _cal_reg.c1 * 65536.0f + (_cal_reg.c3 * dT) / 128;

    if (TEMP < 0)
    {
        // second order temperature compensation when under 20 degrees C
        float T2 = (dT * dT) / 0x80000000;
        float Aux = TEMP * TEMP;
        float OFF2 = 61.0f * Aux / 16.0f;
        float SENS2 = 2.0f * Aux;
        TEMP = TEMP - T2;
        OFF = OFF - OFF2;
        SENS = SENS - SENS2;
    }

    float pressure = (_D1 * SENS / 2097152 - OFF) / 32768;
    float temperature = (TEMP + 2000) * 0.01f;
    update_sensor_status(pressure, temperature);
}

// Calculate Temperature and compensated Pressure in real units (Celsius degrees*100, mbar*100).
void DepthSensor::_calculate_5637()
{
    int32_t dT, TEMP;
    int64_t OFF, SENS;
    int32_t raw_pressure = _D1;
    int32_t raw_temperature = _D2;

    // Formulas from manufacturer datasheet
    // sub -15c temperature compensation is not included

    dT = raw_temperature - (((uint32_t)_cal_reg.c5) << 8);
    TEMP = 2000 + ((int64_t)dT * (int64_t)_cal_reg.c6) / 8388608;
    OFF = (int64_t)_cal_reg.c2 * (int64_t)131072 + ((int64_t)_cal_reg.c4 * (int64_t)dT) / (int64_t)64;
    SENS = (int64_t)_cal_reg.c1 * (int64_t)65536 + ((int64_t)_cal_reg.c3 * (int64_t)dT) / (int64_t)128;

    if (TEMP < 2000)
    {
        // second order temperature compensation when under 20 degrees C
        int32_t T2 = ((int64_t)3 * ((int64_t)dT * (int64_t)dT) / (int64_t)8589934592);
        int64_t aux = (TEMP - 2000) * (TEMP - 2000);
        int64_t OFF2 = 61 * aux / 16;
        int64_t SENS2 = 29 * aux / 16;

        TEMP = TEMP - T2;
        OFF = OFF - OFF2;
        SENS = SENS - SENS2;
    }

    int32_t pressure = ((int64_t)raw_pressure * SENS / (int64_t)2097152 - OFF) / (int64_t)32768;
    float temperature = TEMP * 0.01f;
    update_sensor_status((float)pressure, temperature);
}

// Calculate Temperature and compensated Pressure in real units (Celsius degrees*100, mbar*100).
void DepthSensor::_calculate_5837()
{
    int32_t dT, TEMP;
    int64_t OFF, SENS;
    int32_t raw_pressure = _D1;
    int32_t raw_temperature = _D2;

    // Formulas from manufacturer datasheet
    // sub -15c temperature compensation is not included

    dT = raw_temperature - (((uint32_t)_cal_reg.c5) << 8);
    TEMP = 2000 + ((int64_t)dT * (int64_t)_cal_reg.c6) / 8388608;
    OFF = (int64_t)_cal_reg.c2 * (int64_t)65536 + ((int64_t)_cal_reg.c4 * (int64_t)dT) / (int64_t)128;
    SENS = (int64_t)_cal_reg.c1 * (int64_t)32768 + ((int64_t)_cal_reg.c3 * (int64_t)dT) / (int64_t)256;

    if (TEMP < 2000)
    {
        // second order temperature compensation when under 20 degrees C
        int32_t T2 = ((int64_t)3 * ((int64_t)dT * (int64_t)dT) / (int64_t)8589934592);
        int64_t aux = (TEMP - 2000) * (TEMP - 2000);
        int64_t OFF2 = 3 * aux / 2;
        int64_t SENS2 = 5 * aux / 8;

        TEMP = TEMP - T2;
        OFF = OFF - OFF2;
        SENS = SENS - SENS2;
    }

    int32_t pressure = ((int64_t)raw_pressure * SENS / (int64_t)2097152 - OFF) / (int64_t)8192;
    pressure = pressure * 10; // MS5837 only reports to 0.1 mbar
    float temperature = TEMP * 0.01f;

    update_sensor_status((float)pressure, temperature);
}

/* Check that the baro value is valid by using a mean filter. If the
 * value is further than filtrer_range from mean value, it is
 * rejected.
 */
bool DepthSensor::pressure_ok(float press)
{

    if (isinf(press) || isnan(press))
    {
        return false;
    }

    if (filter_range <= 0)
    {
        return true;
    }

    bool ret = true;
    if (is_zero(_mean_pressure))
    {
        _mean_pressure = press;
    }
    else
    {
        const float d = fabsf(_mean_pressure - press) / (_mean_pressure + press); // diff divide by mean value in percent ( with the * 200.0f on later line)
        float koeff = FILTER_KOEF;

        if (d * 200.0f > filter_range)
        { // check the difference from mean value outside allowed range
            // printf("\nBaro pressure error: mean %f got %f\n", (double)_mean_pressure, (double)press );
            ret = false;
            koeff /= (d * 10.0f); // 2.5 and more, so one bad sample never change mean more than 4%
            _error_count++;
        }
        _mean_pressure = _mean_pressure * (1 - koeff) + press * koeff; // complimentary filter 1/k
    }
    return ret;
}

void DepthSensor::update_sensor_status(float pressure, float temperature)
{
    uint32_t now = AP_HAL::millis();

    // check for changes in data values
    if (!is_equal(depth_sensor_status.pressure, pressure) || !is_equal(depth_sensor_status.temperature, temperature))
    {
        depth_sensor_status.last_change_ms = now;
    }

    // update readings
    depth_sensor_status.pressure = pressure;
    depth_sensor_status.temperature = temperature;
    depth_sensor_status.last_update_ms = now;
}

void DepthSensor::update_healthy_flag(void)
{

    // consider a sensor as healthy if it has had an update in the
    // last 0.5 seconds and values are non-zero and have changed within the last 2 seconds
    const uint32_t now = AP_HAL::millis();
    depth_sensor_status.healthy =
        (now - depth_sensor_status.last_update_ms < BARO_TIMEOUT_MS) &&
        (now - depth_sensor_status.last_change_ms < BARO_DATA_CHANGE_TIMEOUT_MS) &&
        !is_zero(depth_sensor_status.pressure);

    if (depth_sensor_status.temperature < -200 ||
        depth_sensor_status.temperature > 200)
    {
        // if temperature is way out of range then we likely have bad
        // data from the sensor, treat is as unhealthy. This is done
        // so SPI sensors which have no data validity checking can
        // mark a sensor unhealthy
        depth_sensor_status.healthy = false;
    }
}

// void DepthSensor::calibrate()
// {
// start by assuming all sensors are calibrated (for healthy() test)
//     depth_sensor_status.calibrated = true;
//     depth_sensor_status.alt_ok = true;

//     if (hal.util->was_watchdog_reset())
//     {
//         simple_sub.gcs().send_text(MAV_SEVERITY_INFO, "Baro: skipping calibration");
//         return;
//     }
//     simple_sub.gcs().send_text(MAV_SEVERITY_INFO, "Calibrating barometer");

//     // reset the altitude offset when we calibrate. The altitude
//     // offset is supposed to be for within a flight
//     _alt_offset.set_and_save(0);

//     // let the barometer settle for a full second after startup
//     // the MS5611 reads quite a long way off for the first second,
//     // leading to about 1m of error if we don't wait
//     for (uint8_t i = 0; i < 10; i++)
//     {
//         uint32_t tstart = AP_HAL::millis();
//         do
//         {
//             update();
//             if (AP_HAL::millis() - tstart > 500)
//             {
//                 AP_BoardConfig::config_error("Baro: unable to calibrate");
//             }
//             hal.scheduler->delay(10);
//         } while (!healthy());
//         hal.scheduler->delay(100);
//     }

//     // now average over 5 values for the ground pressure settings
//     float sum_pressure[BARO_MAX_INSTANCES] = {0};
//     uint8_t count[BARO_MAX_INSTANCES] = {0};
//     const uint8_t num_samples = 5;

//     for (uint8_t c = 0; c < num_samples; c++)
//     {
//         uint32_t tstart = AP_HAL::millis();
//         do
//         {
//             update();
//             if (AP_HAL::millis() - tstart > 500)
//             {
//                 AP_BoardConfig::config_error("Baro: unable to calibrate");
//             }
//         } while (!healthy());
//         for (uint8_t i = 0; i < _num_sensors; i++)
//         {
//             if (healthy(i))
//             {
//                 sum_pressure[i] += sensors[i].pressure;
//                 count[i] += 1;
//             }
//         }
//         hal.scheduler->delay(100);
//     }
//     for (uint8_t i = 0; i < _num_sensors; i++)
//     {
//         if (count[i] == 0)
//         {
//             sensors[i].calibrated = false;
//         }
//         else
//         {
//             if (save)
//             {
//                 sensors[i].ground_pressure.set_and_save(sum_pressure[i] / count[i]);
//             }
//         }
//     }

//     _guessed_ground_temperature = get_external_temperature();

//     // panic if all sensors are not calibrated
//     uint8_t num_calibrated = 0;
//     for (uint8_t i = 0; i < _num_sensors; i++)
//     {
//         if (sensors[i].calibrated)
//         {
//             BARO_SEND_TEXT(MAV_SEVERITY_INFO, "Barometer %u calibration complete", i + 1);
//             num_calibrated++;
//         }
//     }
//     if (num_calibrated)
//     {
//         return;
//     }
//     AP_BoardConfig::config_error("AP_Baro: all sensors uncalibrated");
// }

float DepthSensor::get_pressure(void) const
{
    return depth_sensor_status.pressure;
}

float DepthSensor::get_temprature(void) const
{
    return depth_sensor_status.temperature;
}