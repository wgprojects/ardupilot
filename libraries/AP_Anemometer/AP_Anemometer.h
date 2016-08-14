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
    virtual int16_t           get_anglecd() = 0;

    // speed in knots
    virtual float           get_speed() = 0;
	
	 // angle sensor - raw counts, and current calibration offset
    AP_Int16           get_angle_raw() { return _dir_raw_counts; }
	AP_Int16           get_angle_offset() { return _dir_raw_counts_cal; }

    static const struct AP_Param::GroupInfo        var_info[];

protected:
	uint32_t                            _last_update; // in ms
	AP_Int16                            _anglecd;
    AP_Float                            _speed;

	AP_Int16                            _dir_raw_counts;
	AP_Int16                            _dir_raw_counts_cal;
	
private:
    
};


#include "AP_Anemometer_Custom.h"
#include "AP_Anemometer_HIL.h"

#endif // __AP_ANEM_H__
