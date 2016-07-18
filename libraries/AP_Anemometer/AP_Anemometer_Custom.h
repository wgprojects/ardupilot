/// -*- tab-width: 4; Mode: C++; c-basic-offset: 4; indent-tabs-mode: nil -*-
#ifndef __AP_ANEM_CUSTOM_H__
#define __AP_ANEM_CUSTOM_H__

#define DIR_FILTER_SIZE 2

#include "AP_Anemometer.h"
#include <AverageFilter.h>

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
    float           get_anglecd();
    float           get_speed();

private:
    int32_t         RawPress;
    int32_t         RawTemp;
    uint8_t			_count;

    AverageFilterInt32_Size4        _temp_filter;

};

#endif // __AP_ANEM_CUSTOM_H__
