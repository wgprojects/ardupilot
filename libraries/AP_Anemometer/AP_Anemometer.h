/// -*- tab-width: 4; Mode: C++; c-basic-offset: 4; indent-tabs-mode: nil -*-

#ifndef __AP_ANEM_H__
#define __AP_ANEM_H__

#include <AP_Param.h>
#include <Filter.h>
#include <DerivativeFilter.h>

class AP_Anemometer
{
public:
    AP_Anemometer() :
		_last_update(0)
    {
       
		AP_Param::setup_object_defaults(this, var_info);
    }

    
    // angle in centidegrees.
    virtual float           get_anglecd() = 0;

    // speed in (what units?)
    virtual float           get_speed() = 0;
	

    static const struct AP_Param::GroupInfo        var_info[];

protected:
	uint32_t                            _last_update; // in ms
	
private:
    AP_Float                            _anglecd;
    AP_Float                            _speed;

};


#include "AP_Anemometer_Custom.h"
#include "AP_Anemometer_HIL.h"

#endif // __AP_ANEM_H__
