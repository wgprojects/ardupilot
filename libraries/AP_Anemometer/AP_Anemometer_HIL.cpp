/// -*- tab-width: 4; Mode: C++; c-basic-offset: 4; indent-tabs-mode: nil -*-

#include <AP_Anemometer.h>
#include "AP_Anemometer_HIL.h"
#include <AP_HAL.h>

extern const AP_HAL::HAL& hal;

// Public Methods //////////////////////////////////////////////////////////////
bool AP_Anemometer_HIL::init()
{
    //_flags.healthy = false;
    return true;
}

// Read the sensor. This is a state machine
uint8_t AP_Anemometer_HIL::read()
{
    uint8_t result = 0;

    if (_count != 0) {
        hal.scheduler->suspend_timer_procs();
        result = 1;
        
        hal.scheduler->resume_timer_procs();
    }

    return result;
}
void AP_Anemometer_HIL::setHIL(float wind_angle, float wind_speed)
{
	
}
float AP_Anemometer_HIL::get_anglecd() {
    return 0;
}

float AP_Anemometer_HIL::get_speed() {
    return 0;
}
