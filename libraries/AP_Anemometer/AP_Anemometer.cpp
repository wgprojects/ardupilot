/// -*- tab-width: 4; Mode: C++; c-basic-offset: 4; indent-tabs-mode: nil -*-
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

/*
 *       APM_Anemometer.cpp - wind speed and direction driver
 *
 */

#include <AP_Math.h>
#include <AP_Common.h>
#include <AP_Anemometer.h>
#include <AP_HAL.h>

extern const AP_HAL::HAL& hal;

// table of user settable parameters
const AP_Param::GroupInfo AP_Anemometer::var_info[] PROGMEM = {
    // @Param: APPARENT_WIND_DIR
    // @DisplayName: apparent wind direction
    // @Description: Wind angle relative to the bow of the boat
    // @Increment: 1
    AP_GROUPINFO("AP_WIND_DIR", 0, AP_Anemometer, _anglecd, 0),

    // @Param: APPARENT_WIND_SPEED
    // @DisplayName: apparent wind speed
    // @Description: wind speed relative to the boat ("as masured")
    // @Increment: 1
    AP_GROUPINFO("AP_WIND_SPEED", 1, AP_Anemometer, _speed, 0),


    AP_GROUPEND
};


// float AP_Anemometer_Custom::get_anglecd() {
    // return 0;
// }

// float AP_Anemometer_Custom::get_speed() {
    // return 0;
// }

