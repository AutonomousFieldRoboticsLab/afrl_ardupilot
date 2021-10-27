#pragma once

#include <AP_Common/AP_Common.h>
#include <AP_HAL/AP_HAL.h>
#include <AP_HAL/I2CDevice.h>
#include <AP_HAL/SPIDevice.h>
#include <AP_Math/AP_Math.h>

class AuxiliaryBus;
class AuxiliaryBusSlave;
class AP_InertialSensor;
class Magnetometer_BusDriver_Auxiliary;

class Magnetometer
{
public:
    /* Probe for AK8963 on auxiliary bus of MPU9250, connected through SPI */
    static Magnetometer *probe_mpu9250(uint8_t mpu9250_instance,
                                       enum Rotation rotation);

    static constexpr const char *name = "AK8963";

    virtual ~Magnetometer();

    void read();
    Vector3f get_magentic_field();

    enum DevTypes
    {
        DEVTYPE_HMC5883_OLD = 0x01,
        DEVTYPE_HMC5883 = 0x07,
        DEVTYPE_LSM303D = 0x02,
        DEVTYPE_AK8963 = 0x04,
        DEVTYPE_BMM150 = 0x05,
        DEVTYPE_LSM9DS1 = 0x06,
        DEVTYPE_LIS3MDL = 0x08,
        DEVTYPE_AK09916 = 0x09,
        DEVTYPE_IST8310 = 0x0A,
        DEVTYPE_ICM20948 = 0x0B,
        DEVTYPE_MMC3416 = 0x0C,
        DEVTYPE_QMC5883L = 0x0D,
        DEVTYPE_MAG3110 = 0x0E,
        DEVTYPE_SITL = 0x0F,
        DEVTYPE_IST8308 = 0x10,
        DEVTYPE_RM3100 = 0x11,
    };

    struct mag_state
    {
        AP_Int8 external;
        bool healthy;
        bool registered;
        AP_Int8 orientation;
        AP_Vector3f offset;
        AP_Vector3f diagonals;
        AP_Vector3f offdiagonals;
        AP_Float scale_factor;

        // device id detected at init.
        // saved to eeprom when offsets are saved allowing ram &
        // eeprom values to be compared as consistency check
        AP_Int32 dev_id;
        int32_t detected_dev_id;
        int32_t expected_dev_id;

        // factors multiplied by throttle and added to compass outputs
        AP_Vector3f motor_compensation;

        // latest compensation added to compass
        Vector3f motor_offset;

        // corrected magnetic field strength
        Vector3f field;

        // when we last got data
        uint32_t last_update_ms;
        uint32_t last_sent_ms;

        // board specific orientation
        enum Rotation rotation;

        // accumulated samples, protected by _sem, used by AP_Compass_Backend
        Vector3f accum;
        uint32_t accum_count;
    };

    mag_state state_;

private:
    Magnetometer(Magnetometer_BusDriver_Auxiliary *bus, enum Rotation rotation);

    bool init();
    void _make_factory_sensitivity_adjustment(Vector3f &field) const;
    void _make_adc_sensitivity_adjustment(Vector3f &field) const;

    bool _reset();
    bool _setup_mode();
    bool _check_id();
    bool _calibrate();

    void _update();

    Magnetometer_BusDriver_Auxiliary *_bus;

    float _magnetometer_ASA[3]{0, 0, 0};

    bool _initialized;
    enum Rotation _rotation;
    float filter_range_;
    /*
     * A compass measurement is expected to pass through the following functions:
     * 1. rotate_field - this rotates the measurement in-place from sensor frame
     *      to body frame
     * 2. publish_raw_field - this provides an uncorrected point-sample for
     *      calibration libraries
     * 3. correct_field - this corrects the measurement in-place for hard iron,
     *      soft iron, motor interference, and non-orthagonality errors
     * 4. publish_filtered_field - legacy filtered magnetic field
     *
     * All those functions expect the mag field to be in milligauss.
     */

    void rotate_field(Vector3f &mag);
    void publish_raw_field(const Vector3f &mag);
    void correct_field(Vector3f &mag);
    void publish_filtered_field(const Vector3f &mag);
    void set_last_update_usec(uint32_t last_update);

    void accumulate_sample(Vector3f &field,
                           uint32_t max_samples = 10);
    void drain_accumulated_samples(const Vector3f *scale = NULL);

    bool register_compass(int32_t dev_id);

    // set rotation of an instance
    void set_rotation(enum Rotation rotation);

    // get board orientation (for SITL)
    enum Rotation get_board_orientation(void) const;

    // Check that the compass field is valid by using a mean filter on the vector length
    bool field_ok(const Vector3f &field);

    uint32_t get_error_count() const { return _error_count; }

    void apply_corrections(Vector3f &mag, uint8_t i);

    // mean field length for range filter
    float _mean_field_length;
    // number of dropped samples. Not used for now, but can be usable to choose more reliable sensor
    uint32_t _error_count;
};

class Magnetometer_BusDriver_Auxiliary
{
public:
    Magnetometer_BusDriver_Auxiliary(AP_InertialSensor &ins, uint8_t backend_id,
                                     uint8_t backend_instance, uint8_t addr);
    ~Magnetometer_BusDriver_Auxiliary();

    bool block_read(uint8_t reg, uint8_t *buf, uint32_t size);
    bool register_read(uint8_t reg, uint8_t *val);
    bool register_write(uint8_t reg, uint8_t val);

    AP_HAL::Device::PeriodicHandle register_periodic_callback(uint32_t period_usec, AP_HAL::Device::PeriodicCb);

    AP_HAL::Semaphore *get_semaphore();

    bool configure();
    bool start_measurements();

    // set device type within a device class
    void set_device_type(uint8_t devtype);

    // return 24 bit bus identifier
    uint32_t get_bus_id(void) const;

private:
    AuxiliaryBus *_bus;
    AuxiliaryBusSlave *_slave;
    bool _started;
};
