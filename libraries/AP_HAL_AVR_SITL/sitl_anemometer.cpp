/*
  SITL handling

  This simulates a anemometer

  Andrew Tridgell November 2011
 */

#include <AP_HAL.h>
#include <AP_Math.h>

#if CONFIG_HAL_BOARD == HAL_BOARD_AVR_SITL

#include "AP_HAL_AVR_SITL.h"

using namespace AVR_SITL;

extern const AP_HAL::HAL& hal;

#include <unistd.h>
#include <stdio.h>
#include <stdlib.h>
#include <stdint.h>
#include <math.h>

/*
  setup the anemometer with new input
 */
void SITL_State::_update_anemometer(float wind_direction, float wind_speed, float model_speedN, float model_speedE, float model_speedD)
{
	static uint32_t last_update;

	if (_anemometer == NULL) {
		// this sketch doesn't use a anemometer
		return;
	}

        if (_sitl->anemometer_disable) {
            // anemometer is disabled
            return;
        }

	// 80Hz, to match the real APM2 anemometer
        uint32_t now = hal.scheduler->millis();
	if ((now - last_update) < 12) {
		return;
	}
	last_update = now;

    Vector3f model_v = Vector3f(model_speedN, model_speedE, model_speedD);
    Vector3f wind_v = Vector3f(cosf(radians(wind_direction)), sin(radians(wind_direction)), 0) * wind_speed;

    Vector3f apparent = wind_v - model_v;

	// sim_alt += _sitl->baro_drift * now / 1000;
	// sim_alt += _sitl->baro_noise * _rand_float();

	// // add baro glitch
	// sim_alt += _sitl->baro_glitch;

	float model_dir = atan2f(model_speedN, model_speedE);

    float vane_dir = atan2f(apparent.y, apparent.x) - model_dir;

	_anemometer->setHIL(vane_dir, 1);
}

#endif
