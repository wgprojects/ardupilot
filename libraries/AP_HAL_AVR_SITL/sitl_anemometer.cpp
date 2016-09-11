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
void SITL_State::_update_anemometer(float wind_direction, float wind_speed, float model_speedN, float model_speedE, float model_speedD, float heading)
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

	Vector3f model_v = Vector3f(model_speedN, model_speedE, 0);
	// wind comes FROM direction, reverse vector for velocity
	Vector3f wind_v = Vector3f(-cosf(radians(wind_direction)), -sinf(radians(wind_direction)), 0) * wind_speed;

	Vector3f apparent = wind_v - model_v;

	// reverse velocity back into wind from-direction
	float apparent_dir = degrees(atan2f(-apparent.y, -apparent.x));
	float apparent_speed = apparent.length();

	float vane_dir = apparent_dir - heading;

	// TODO: add noise similar to other sensors
	// sim_alt += _sitl->baro_noise * _rand_float();

	if (vane_dir <= -180)
		vane_dir += 360;
	else if (vane_dir > 180)
		vane_dir -= 360;

	// printf ("DEBUG: %s %s wind_dir: %.1f wind_speed: %.2f model_dir: %.1f model_speed: %.2f apparent_dir: %.1f apparent_speed: %.2f vane_dir: %.1f\n", __FILE__, __FUNCTION__,
		// wind_direction, wind_speed, degrees(atan2(model_speedN, model_speedE)), model_v.length(), apparent_dir, apparent_speed, vane_dir);

	// printf ("DEBUG:\n");

	_anemometer->setHIL(vane_dir, apparent_speed);
}

#endif
