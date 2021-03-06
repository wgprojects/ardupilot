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
 *       AP_Anemometer_Custom.cpp - custom hacklab anemometer - wind speed and direction sensor.
 
 *       Variables:
 *               RawTemp : Raw temperature data
 *               RawPress : Raw pressure data
 *
 *               Temp : Calculated temperature (in 0.1�C units)
 *               Press : Calculated pressure   (in Pa units)
 *
 *       Methods:
 *               Init() : Initialization of I2C and read sensor calibration data
 *               Read() : Read sensor data and calculate Temperature and Pressure
 *                        This function is optimized so the main host don�t need to wait
 *                                You can call this function in your main loop
 *                                It returns a 1 if there are new data.
 *
 *       Internal functions:
 *               Command_ReadTemp(): Send commando to read temperature
 *               Command_ReadPress(): Send commando to read Pressure
 *               ReadTemp() : Read temp register
 *               ReadPress() : Read press register
 *
 *
 */

// AVR LibC Includes
#include <inttypes.h>

#include <AP_Common.h>
#include <AP_Math.h>            // ArduPilot Mega Vector/Matrix math Library

#include <AP_HAL.h>
#include "AP_Anemometer_Custom.h"

extern const AP_HAL::HAL& hal;

#define BMP085_ADDRESS 0x77  //(0xEE >> 1)
#define BMP085_EOC 30        // End of conversion pin PC7 on APM1

// the apm2 hardware needs to check the state of the
// chip using a direct IO port
// On APM2 prerelease hw, the data ready port is hooked up to PE7, which
// is not available to the arduino digitalRead function.
#if CONFIG_HAL_BOARD == HAL_BOARD_APM1
#define BMP_DATA_READY() hal.gpio->read(BMP085_EOC)
#else
// No EOC connection from Baro
// Use times instead.
// Temp conversion time is 4.5ms
// Pressure conversion time is 25.5ms (for OVERSAMPLING=3)
#define BMP_DATA_READY() (BMP085_State == 0 ? hal.scheduler->millis() > (_last_temp_read_command_time + 5) : hal.scheduler->millis() > (_last_press_read_command_time + 26))
#endif

// oversampling 3 gives 26ms conversion time. We then average
#define OVERSAMPLING 3

// Public Methods //////////////////////////////////////////////////////////////
bool AP_Anemometer_Custom::init()
{
    tmp = 0;
	_dir_raw_counts = 1000; //for testing - TODO remove.
	

	// Following code is for setting, GPIO for serial port
	// For AET6010, 6012, magnetic encoder chip the following settings are initialized, clock, Clock Speed, and MISO
	// HAL_GPIO_OUTPUT = 1; 
	// MISO, only to read so, HAL_GPIO_OUTPUT, nothing to write on MISO
	hal.gpio->pinMode(__AP_ANEM_CS, HAL_GPIO_OUTPUT);
	hal.gpio->pinMode(__AP_ANEM_SCK, HAL_GPIO_OUTPUT);
	hal.gpio->pinMode(__AP_ANEM_MISO, HAL_GPIO_INPUT);

	// Now select AET5012, system clock, and MISO  to HIGH. 
	hal.gpio->write(__AP_ANEM_CS, 1);
	hal.gpio->write(__AP_ANEM_SCK, 1);
    
    return true;
}



// Read the sensor using accumulated data
uint8_t AP_Anemometer_Custom::read()
{

        
    //Note: 12 bits for SPIO reading, on wind direction = 4096 decimal

    //Offset = Offset to be deducted from angle reading and passed to getanglecd() 

    int16_t AnemoRdg = 0; //initialize AnemoRdg; 

    //Intializing the  GPIO to read
    hal.gpio->write(__AP_ANEM_CS, 0);

    
    //From timing diagram, chip select is low
    //clock is high 
    hal.gpio->write(__AP_ANEM_SCK, 1); 
    hal.scheduler->delay_microseconds(1);  //clock delay from the timing diagram, 500 ns latency 


    // For first read , clock low; delay; 
    hal.gpio->write(__AP_ANEM_SCK, 0);
    hal.scheduler->delay_microseconds(1);

    hal.gpio->pinMode(__AP_ANEM_MISO, HAL_GPIO_INPUT);
    uint8_t i;
    for (i = 0; i < 12; i++)// Start reading the bits here 
    {

        // clock high to read
        hal.gpio->write(__AP_ANEM_SCK, 1); 
        hal.scheduler->delay_microseconds(1); 

        AnemoRdg = AnemoRdg << 1;
        AnemoRdg = AnemoRdg | (int16_t)hal.gpio->read(__AP_ANEM_MISO);// read MISO
        
        //clock low; 
        hal.gpio->write(__AP_ANEM_SCK, 0);
        hal.scheduler->delay_microseconds(1); // delay the millisecs on clock for 1 us 
    }

    //Done reading, set SCK and CS high
    hal.gpio->write(__AP_ANEM_SCK, 1);
    hal.gpio->write(__AP_ANEM_CS, 1); 
    

    _dir_raw_counts = AnemoRdg + 1; //Set new value
    
    
    //Calculate angle from _dir_raw_counts:
    //Angle in degrees is 365 degrees / 2^12 counts * (reading - calibration)
    //where reading == calibration at angle == 0, or 'dead ahead'
    int32_t dir_temp = (((int32_t)36000) * (_dir_raw_counts - _dir_raw_counts_cal)) >> 12;
    
    //Return +90 is to starboard (right)
    //Return -90 is to port 	 (left)
    if(dir_temp > 18000)
        dir_temp -= 36000;
    _anglecd = (uint16_t)dir_temp;
    
	_last_update = hal.scheduler->millis();
    return 1;
}

int16_t AP_Anemometer_Custom::get_anglecd() {
    return _anglecd;
}

float AP_Anemometer_Custom::get_speed() {
    return 0;
}

// Private functions: /////////////////////////////////////////////////////////


