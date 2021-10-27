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
#include <assert.h>
#include <utility>

#include <AP_Math/AP_Math.h>
#include <AP_HAL/AP_HAL.h>
#include <AP_InertialSensor/AP_InertialSensor_Invensense.h>

#include "Magnetometer.h"
#include "Sub.h"

#define MAG_BOARD_ORIENTATION ROTATION_NONE

#define AK8963_I2C_ADDR 0x0c

#define AK8963_WIA 0x00
#define AK8963_Device_ID 0x48

#define AK8963_HXL 0x03

/* bit definitions for AK8963 CNTL1 */
#define AK8963_CNTL1 0x0A
#define AK8963_CONTINUOUS_MODE1 0x02
#define AK8963_CONTINUOUS_MODE2 0x06
#define AK8963_SELFTEST_MODE 0x08
#define AK8963_POWERDOWN_MODE 0x00
#define AK8963_FUSE_MODE 0x0f
#define AK8963_16BIT_ADC 0x10
#define AK8963_14BIT_ADC 0x00

#define AK8963_CNTL2 0x0B
#define AK8963_RESET 0x01

#define AK8963_ASAX 0x10

#define AK8963_MILLIGAUSS_SCALE 10.0f

struct PACKED sample_regs
{
    int16_t val[3];
    uint8_t st2;
};

extern const AP_HAL::HAL &hal;

Magnetometer::Magnetometer(Magnetometer_BusDriver_Auxiliary *bus, enum Rotation rotation)
    : _bus(bus), _rotation(rotation)
{
}

Magnetometer::~Magnetometer()
{
    delete _bus;
}

Magnetometer *Magnetometer::probe_mpu9250(uint8_t mpu9250_instance,
                                          enum Rotation rotation)
{
    AP_InertialSensor &ins = *AP_InertialSensor::get_singleton();

    Magnetometer_BusDriver_Auxiliary *bus = new Magnetometer_BusDriver_Auxiliary(ins, HAL_INS_MPU9250_SPI, mpu9250_instance, AK8963_I2C_ADDR);
    if (!bus)
    {
        simple_sub.gcs().send_text(MAV_SEVERITY_WARNING, "Auxillary bus is nullptr");
        return nullptr;
    }

    Magnetometer *sensor = new Magnetometer(bus, rotation);
    if (!sensor || !sensor->init())
    {
        simple_sub.gcs().send_text(MAV_SEVERITY_WARNING, " Sensor object is nullptr");

        delete sensor;
        return nullptr;
    }

    return sensor;
}

bool Magnetometer::init()
{
    filter_range_ = 34.0;

    AP_HAL::Semaphore *bus_sem = _bus->get_semaphore();

    if (!bus_sem)
    {
        simple_sub.gcs().send_text(MAV_SEVERITY_WARNING, "Could not get the bus semaphore\n");
        return false;
    }
    _bus->get_semaphore()->take_blocking();

    if (!_bus->configure())
    {
        simple_sub.gcs().send_text(MAV_SEVERITY_WARNING, "AK8963: Could not configure the bus\n");
        goto fail;
    }

    if (!_check_id())
    {
        simple_sub.gcs().send_text(MAV_SEVERITY_WARNING, "AK8963: Wrong id\n");
        goto fail;
    }

    if (!_calibrate())
    {
        simple_sub.gcs().send_text(MAV_SEVERITY_WARNING, "AK8963: Could not read calibration data\n");
        goto fail;
    }

    if (!_setup_mode())
    {
        simple_sub.gcs().send_text(MAV_SEVERITY_WARNING, "AK8963: Could not setup mode\n");
        goto fail;
    }

    if (!_bus->start_measurements())
    {
        simple_sub.gcs().send_text(MAV_SEVERITY_WARNING, "AK8963: Could not start measurements\n");
        goto fail;
    }

    _initialized = true;

    /* register the compass instance in the frontend */
    _bus->set_device_type(DEVTYPE_AK8963);

    state_.expected_dev_id = _bus->get_bus_id();
    state_.detected_dev_id = _bus->get_bus_id();
    state_.dev_id = _bus->get_bus_id();
    state_.registered = true;
    state_.rotation = _rotation;

    bus_sem->give();

    _bus->register_periodic_callback(10000, FUNCTOR_BIND_MEMBER(&Magnetometer::_update, void));

    return true;

fail:
    bus_sem->give();
    return false;
}

void Magnetometer::read()
{
    if (!_initialized)
    {
        simple_sub.gcs().send_text(MAV_SEVERITY_INFO, "magentometer not initialized");
        return;
    }

    drain_accumulated_samples();
}

Vector3f Magnetometer::get_magentic_field()
{
    uint32_t current_time = AP_HAL::millis();
    if (current_time - state_.last_sent_ms > MAGNETOMETER_MESSAGE_RATE_MILLIS)
    {
        // hal.console->printf("current_time: %ld\t last_sent: %ld\r\n", current_time, state_.last_sent_ms);
        read();
    }

    return state_.field;
}

void Magnetometer::_make_adc_sensitivity_adjustment(Vector3f &field) const
{
    static const float ADC_16BIT_RESOLUTION = 0.15f;

    field *= ADC_16BIT_RESOLUTION;
}

void Magnetometer::_make_factory_sensitivity_adjustment(Vector3f &field) const
{
    field.x *= _magnetometer_ASA[0];
    field.y *= _magnetometer_ASA[1];
    field.z *= _magnetometer_ASA[2];
}

void Magnetometer::_update()
{
    struct sample_regs regs;
    Vector3f raw_field;

    if (!_bus->block_read(AK8963_HXL, (uint8_t *)&regs, sizeof(regs)))
    {
        hal.console->printf("Auxillary bus block read fail\r\n");
        return;
    }

    /* Check for overflow. See AK8963's datasheet, section
     * 6.4.3.6 - Magnetic Sensor Overflow. */
    if ((regs.st2 & 0x08))
    {
        hal.console->printf("magnetic sensor overflow\r\n");
        return;
    }

    raw_field = Vector3f(regs.val[0], regs.val[1], regs.val[2]);

    if (is_zero(raw_field.x) && is_zero(raw_field.y) && is_zero(raw_field.z))
    {
        return;
    }

    _make_factory_sensitivity_adjustment(raw_field);
    _make_adc_sensitivity_adjustment(raw_field);
    raw_field *= AK8963_MILLIGAUSS_SCALE;

    // hal.console->printf("Raw field: %f, %f, %f\r\n", raw_field.x, raw_field.y, raw_field.z);

    accumulate_sample(raw_field, 10);
}

bool Magnetometer::_check_id()
{
    for (int i = 0; i < 5; i++)
    {
        uint8_t deviceid = 0;

        /* Read AK8963's id */
        if (_bus->register_read(AK8963_WIA, &deviceid) &&
            deviceid == AK8963_Device_ID)
        {
            return true;
        }
    }

    return false;
}

bool Magnetometer::_setup_mode()
{
    return _bus->register_write(AK8963_CNTL1, AK8963_CONTINUOUS_MODE2 | AK8963_16BIT_ADC);
}

bool Magnetometer::_reset()
{
    return _bus->register_write(AK8963_CNTL2, AK8963_RESET);
}

bool Magnetometer::_calibrate()
{
    /* Enable FUSE-mode in order to be able to read calibration data */
    _bus->register_write(AK8963_CNTL1, AK8963_FUSE_MODE | AK8963_16BIT_ADC);

    uint8_t response[3];

    _bus->block_read(AK8963_ASAX, response, 3);

    for (int i = 0; i < 3; i++)
    {
        float data = response[i];
        _magnetometer_ASA[i] = ((data - 128) / 256 + 1);
    }

    return true;
}

/* AK8963 on an auxiliary bus of IMU driver */
Magnetometer_BusDriver_Auxiliary::Magnetometer_BusDriver_Auxiliary(AP_InertialSensor &ins, uint8_t backend_id,
                                                                   uint8_t backend_instance, uint8_t addr)
{
    /*
     * Only initialize members. Fails are handled by configure or while
     * getting the semaphore
     */
    _bus = ins.get_auxiliary_bus(backend_id, backend_instance);
    if (!_bus)
    {
        simple_sub.gcs().send_text(MAV_SEVERITY_WARNING, "Cannot get bus from inertial sensor");
        return;
    }

    _slave = _bus->request_next_slave(addr);
}

Magnetometer_BusDriver_Auxiliary::~Magnetometer_BusDriver_Auxiliary()
{
    /* After started it's owned by AuxiliaryBus */
    if (!_started)
    {
        delete _slave;
    }
}

bool Magnetometer_BusDriver_Auxiliary::block_read(uint8_t reg, uint8_t *buf, uint32_t size)
{
    if (_started)
    {
        /*
         * We can only read a block when reading the block of sample values -
         * calling with any other value is a mistake
         */
        assert(reg == AK8963_HXL);

        int n = _slave->read(buf);
        return n == static_cast<int>(size);
    }

    int r = _slave->passthrough_read(reg, buf, size);

    return r > 0 && static_cast<uint32_t>(r) == size;
}

bool Magnetometer_BusDriver_Auxiliary::register_read(uint8_t reg, uint8_t *val)
{
    return _slave->passthrough_read(reg, val, 1) == 1;
}

bool Magnetometer_BusDriver_Auxiliary::register_write(uint8_t reg, uint8_t val)
{
    return _slave->passthrough_write(reg, val) == 1;
}

AP_HAL::Semaphore *Magnetometer_BusDriver_Auxiliary::get_semaphore()
{
    return _bus ? _bus->get_semaphore() : nullptr;
}

bool Magnetometer_BusDriver_Auxiliary::configure()
{
    if (!_bus || !_slave)
    {
        return false;
    }
    return true;
}

bool Magnetometer_BusDriver_Auxiliary::start_measurements()
{
    if (_bus->register_periodic_read(_slave, AK8963_HXL, sizeof(sample_regs)) < 0)
    {
        return false;
    }

    _started = true;

    return true;
}

AP_HAL::Device::PeriodicHandle Magnetometer_BusDriver_Auxiliary::register_periodic_callback(uint32_t period_usec, AP_HAL::Device::PeriodicCb cb)
{
    return _bus->register_periodic_callback(period_usec, cb);
}

// set device type within a device class
void Magnetometer_BusDriver_Auxiliary::set_device_type(uint8_t devtype)
{
    _bus->set_device_type(devtype);
}

// return 24 bit bus identifier
uint32_t Magnetometer_BusDriver_Auxiliary::get_bus_id(void) const
{
    return _bus->get_bus_id();
}

void Magnetometer::rotate_field(Vector3f &mag)
{
    mag.rotate(MAG_BOARD_ORIENTATION);
    mag.rotate(state_.rotation);
}

void Magnetometer::publish_raw_field(const Vector3f &mag)
{

    // note that we do not set last_update_usec here as otherwise the
    // EKF and DCM would end up consuming compass data at the full
    // sensor rate. We want them to consume only the filtered fields
    state_.last_update_ms = AP_HAL::millis();
}

void Magnetometer::correct_field(Vector3f &mag)
{

    if (state_.diagonals.get().is_zero())
    {
        state_.diagonals.set(Vector3f(1.0f, 1.0f, 1.0f));
    }

    const Vector3f &offsets = state_.offset.get();
    const Vector3f &diagonals = state_.diagonals.get();
    const Vector3f &offdiagonals = state_.offdiagonals.get();

    // add in the basic offsets
    mag += offsets;

    // apply eliptical correction
    Matrix3f mat(
        diagonals.x, offdiagonals.x, offdiagonals.y,
        offdiagonals.x, diagonals.y, offdiagonals.z,
        offdiagonals.y, offdiagonals.z, diagonals.z);

    mag = mat * mag;

#if COMPASS_MOT_ENABLED
    const Vector3f &mot = state.motor_compensation.get();
    /*
      calculate motor-power based compensation
      note that _motor_offset[] is kept even if compensation is not
      being applied so it can be logged correctly
    */
    state.motor_offset.zero();
    if (_compass._per_motor.enabled() && i == 0)
    {
        // per-motor correction is only valid for first compass
        _compass._per_motor.compensate(state.motor_offset);
    }
    else if (_compass._motor_comp_type == AP_COMPASS_MOT_COMP_THROTTLE)
    {
        state.motor_offset = mot * _compass._thr;
    }
    else if (_compass._motor_comp_type == AP_COMPASS_MOT_COMP_CURRENT)
    {
        AP_BattMonitor &battery = AP::battery();
        float current;
        if (battery.current_amps(current))
        {
            state.motor_offset = mot * current;
        }
    }

    /*
      we apply the motor offsets after we apply the eliptical
      correction. This is needed to match the way that the motor
      compensation values are calculated, as they are calculated based
      on final field outputs, not on the raw outputs
    */
    mag += state.motor_offset;
#endif // COMPASS_MOT_ENABLED
}

void Magnetometer::accumulate_sample(Vector3f &field, uint32_t max_samples)
{
    /* rotate raw_field from sensor frame to body frame */
    rotate_field(field);

    /* publish raw_field (uncorrected point sample) for calibration use */
    publish_raw_field(field);

    /* correct raw_field for known errors */
    correct_field(field);

    if (!field_ok(field))
    {
        hal.console->printf("Field not OK \r\n");
        return;
    }

    // hal.console->printf("Magnetic Field: %f, %f, %f", field.x, field.y, field.z);

    state_.accum += field;
    state_.accum_count++;
    if (max_samples && state_.accum_count >= max_samples)
    {
        state_.accum_count /= 2;
        state_.accum /= 2;
    }
}

void Magnetometer::drain_accumulated_samples(const Vector3f *scaling)
{

    if (state_.accum_count == 0)
    {
        return;
    }

    if (scaling)
    {
        state_.accum *= *scaling;
    }
    state_.accum /= state_.accum_count;

    publish_filtered_field(state_.accum);
    hal.console->printf("Mag field: %f %f %f\r\n", state_.field.x, state_.field.y, state_.field.z);

    state_.accum.zero();
    state_.accum_count = 0;
}

/*
  copy latest data to the frontend from a backend
 */
void Magnetometer::publish_filtered_field(const Vector3f &mag)
{
    state_.field = mag;
    state_.last_update_ms = AP_HAL::millis();
    state_.last_sent_ms = state_.last_update_ms;
}

static constexpr float FILTER_KOEF = 0.1f;

/* Check that the compass value is valid by using a mean filter. If
 * the value is further than filtrer_range from mean value, it is
 * rejected.
 */
bool Magnetometer::field_ok(const Vector3f &field)
{

    if (field.is_inf() || field.is_nan())
    {
        return false;
    }

    if (filter_range_ <= 0)
    {
        return true;
    }

    const float length = field.length();

    if (is_zero(_mean_field_length))
    {
        _mean_field_length = length;
        return true;
    }

    bool ret = true;
    const float d = fabsf(_mean_field_length - length) / (_mean_field_length + length); // diff divide by mean value in percent ( with the *200.0f on later line)
    float koeff = FILTER_KOEF;

    if (d * 200.0f > filter_range_)
    { // check the difference from mean value outside allowed range
        // printf("\nCompass field length error: mean %f got %f\n", (double)_mean_field_length, (double)length );
        ret = false;
        koeff /= (d * 10.0f); // 2.5 and more, so one bad sample never change mean more than 4%
        _error_count++;
    }
    _mean_field_length = _mean_field_length * (1 - koeff) + length * koeff; // complimentary filter 1/k

    return ret;
}
