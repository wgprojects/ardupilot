/// -*- tab-width: 4; Mode: C++; c-basic-offset: 4; indent-tabs-mode: nil -*-
#ifndef __AP_ANEM_CUSTOM_H__
#define __AP_ANEM_CUSTOM_H__

// CS = 55 
// SCK = 52 
// MISO = 50
// Notes:  MISO used only for reading no writes on this, so dont need to set to low. So, before reading anemometer 
// magnetic sensor AERT08, set CS(chip select AET6012) chip selects to write on bus, SCK to low, leave MISO to high


//PB7 13
#define __AP_ANEM_CS 13
//PL4 45
#define __AP_ANEM_SCK 45
//PL5 44
#define __AP_ANEM_MISO 44

// All AEAT-6010/6012 Magnetic Encoder settings for Ardupilot, the numbers that match on the chip are different 

// The actual wiring matches to a different set, using rhe follwing mapping(?)pins_arduino_mega.cpp, use 12 bits



#define DIR_FILTER_SIZE 2

#include "AP_Anemometer.h"
#include <AverageFilter.h>
#include <AP_HAL.h>

class AP_Anemometer_Custom : public AP_Anemometer
{
public:
    AP_Anemometer_Custom() :
        RawPress(0),
        RawTemp(0),
        _count(0)
    {
        
    };       // Constructor


    /* AP_Anemometer public interface: */
    bool            init();
    uint8_t         read();
    int16_t         get_anglecd();
    float           get_speed();

private:
    int32_t         RawPress;
    int32_t         RawTemp;
    uint8_t			_count;
	uint32_t 		tmp;
    
    // SPI device
    AP_HAL::SPIDeviceDriver *_spi;

    //AverageFilterInt32_Size4        _temp_filter;

};

#endif // __AP_ANEM_CUSTOM_H__
