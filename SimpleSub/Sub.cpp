#include "Sub.h"
#include "SimpleGCS.h"
#include "Config.h"

const AP_HAL::HAL &hal = AP_HAL::get_HAL();

SimpleSub::SimpleSub()
{

    if (_singleton)
    {
        AP_HAL::panic("Too many Vehicles");
    }
    _singleton = this;

    last_imu_message_send_time_ = 0;
    last_pressure_sent_time_ = 0;
    last_performance_report_time_ = 0;
}

SimpleSub simple_sub;

SimpleSub *SimpleSub::_singleton = nullptr;

SimpleSub *SimpleSub::get_singleton()
{
    return _singleton;
}
