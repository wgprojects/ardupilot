// -*- tab-width: 4; Mode: C++; c-basic-offset: 4; indent-tabs-mode: nil -*-

/// @file    AP_L2_Control.h
/// @brief   Tacking algoritm for sailing. This is a instance of an
/// AP_L1_Control class

/*
 * Writted by William Gibson 2016
 *
 */

#ifndef AP_L2_Control_H
#define AP_L2_Control_H

#include <AP_Math.h>
#include <AP_AHRS.h>
#include <AP_Param.h>
#include <AP_Navigation.h>
#include <../AP_L1_Control/AP_L1_Control.h>
#include <AP_Anemometer.h>

class AP_L2_Control :  public AP_Navigation {
	
enum BeatingTack {
	NONE     = 0,
	PORT,
	STARBOARD
};

	
public:
	AP_L2_Control(AP_AHRS &ahrs, AP_L1_Control &L1, AP_Anemometer &anem) :
		_ahrs(ahrs), _L1(L1), _anem(anem)
		{
			AP_Param::setup_object_defaults(this, var_info);
			
			tack = NONE;
		}


	int32_t nav_roll_cd(void) const;
	float lateral_acceleration(void) const;

	// return the desired track heading angle(centi-degrees)
	int32_t nav_bearing_cd(void) const;
	
	// return the heading error angle (centi-degrees) +ve to left of track
	int32_t bearing_error_cd(void) const;

	float crosstrack_error(void) const;

	int32_t target_bearing_cd(void) const;
	float turn_distance(float wp_radius) const;
	float turn_distance(float wp_radius, float turn_angle) const;
	void update_waypoint(const struct Location &prev_WP, const struct Location &next_WP);
	void update_loiter(const struct Location &center_WP, float radius, int8_t loiter_direction);
	void update_heading_hold(int32_t navigation_heading_cd);
	void update_level_flight(void);
	bool reached_loiter_target(void);

    // set the default NAV_L1_PERIOD
    void set_default_period(float period) {
        _L1.set_default_period(period);
    }


	// this supports the NAVL2_* user settable parameters
    static const struct AP_Param::GroupInfo var_info[];

private:
	
	AP_L1_Control &_L1;
	
	// reference to the AHRS object
    AP_AHRS &_ahrs;
	
    AP_Anemometer &_anem;

	AP_Float _close_hauled_angle;
	enum BeatingTack tack;
	
	struct Location lastRequestedDestination;
	
	bool _WPcircle;
	
	// target bearing in centi-degrees from last update
	int32_t _target_bearing_cd;
	int32_t _test_angle;
	static int8_t opt;


};
#endif //AP_L2_Control_H
