// -*- tab-width: 4; Mode: C++; c-basic-offset: 4; indent-tabs-mode: nil -*-

#include <AP_HAL.h>
#include "AP_L2_Control.h"

extern const AP_HAL::HAL& hal;

// table of user settable parameters
const AP_Param::GroupInfo AP_L2_Control::var_info[] PROGMEM = {
    // @Param: CLOSE_ANGLE
    // @DisplayName: Close-hauled Min Angle
    // @Description: Angle in degrees - the closest this boat should sail to the wind. Any closer, and tacking begins.
	// @Units: degrees
	// @Range: 20-60
	// @Increment: 1
    AP_GROUPINFO("CLOSE_ANGLE",    0, AP_L2_Control, _close_hauled_angle, 40),
	
  
    AP_GROUPEND
};



// update L1 control for waypoint navigation
void AP_L2_Control::update_waypoint(const struct Location &prev_WP, const struct Location &next_WP)
{
	struct Location _current_loc;

	//Get current position and velocity
    _ahrs.get_position(_current_loc);

	_target_bearing_cd = get_bearing_cd(_current_loc, next_WP);
	
	
	int16_t wind_dir_relative_cd = _anem.get_anglecd(); 
	
	int16_t close_hauled_cd = 100 * _close_hauled_angle;
	close_hauled_cd = constrain_int16(close_hauled_cd, 2000, 6000);
	
	int16_t wind_dir_absolute_cd = _ahrs.yaw_sensor + wind_dir_relative_cd;
	int16_t steer_port_angle = wind_dir_absolute_cd - close_hauled_cd;
	int16_t steer_starboard_angle = wind_dir_absolute_cd + close_hauled_cd;
	
	int16_t port_error = _target_bearing_cd - steer_port_angle; 			//When destination is in irons, this is positive.
	int16_t starboard_error = _target_bearing_cd - steer_starboard_angle;	//When destination is in irons, this is positive.
	
	if(starboard_error < 0 || port_error < 0)
	{
		//The destination can be sailed to directly.
		//Note, sailing directly downwind is *possible*, but not necessarily the fastest. TODO: Consider tacking in this regime as well.
		
		//Forget our previous tack choice.
		tack = NONE; 
		
		//Destination is NOT in irons - sail directly.
		_L1.update_waypoint(prev_WP, next_WP);
	}	
	else
	{
		//The destination is within our 'no-go' zone; we would be 'in irons' if we tried to sail there.
		//Prevent indecision by picking the best tack, then sticking with it!
		//Eventually the destination will leave our no-go zone, and we will navigate to the waypoint.
		
		//If we are navigating to a NEW waypoint, forget our previous tack choice (it probably doesn't apply)
		if(get_distance(next_WP, lastRequestedDestination) > 0.1) //More than 10cm distance
		{
			lastRequestedDestination = next_WP;
			tack = NONE;
		}
		
		//Need to pick a tack - choose the 'closest' to our destination.
		if(tack == NONE) 
		{
			if(starboard_error < port_error)
				tack = STARBOARD;
			else
				tack = PORT;
		}
		
		//Now we're either on a PORT or STARBOARD tack, sailing as close to the wind as possible.
		if(tack == STARBOARD)
			_L1.update_heading_hold(steer_starboard_angle);
		else //tack == PORT
			_L1.update_heading_hold(steer_port_angle);
		
			
	}
}
// update L1 control for loitering
void AP_L2_Control::update_loiter(const struct Location &center_WP, float radius, int8_t loiter_direction)
{
	struct Location _current_loc;

	//Get current position and velocity
    _ahrs.get_position(_current_loc);


	//Calculate the NE position of the aircraft relative to WP A
    Vector2f A_air = location_diff(center_WP, _current_loc);

	//Calculate radial position error
	float xtrackErrCirc = A_air.length() - radius; // Radial distance from the loiter circle

	
	// Perform switchover between 'capture' and 'in irons' modes.
	// Only sail 'capture' mode if outside the circle
	if (xtrackErrCirc > 0.0f ) {
		_WPcircle = false;
		update_waypoint(_current_loc, center_WP); //Tack to the center of the circle.
		
	} else {
		//Run L1 controller - aim to the bearing which is directly 'in irons'.
		_WPcircle = true;
		
		int16_t wind_dir_relative_cd = _anem.get_anglecd(); 
		_L1.update_heading_hold(_ahrs.yaw_sensor + wind_dir_relative_cd);
	}
}
	

int32_t AP_L2_Control::nav_roll_cd(void) const
{
	return _L1.nav_roll_cd();
}
float AP_L2_Control::lateral_acceleration(void) const
{
	return _L1.lateral_acceleration();
}

	
int32_t AP_L2_Control::nav_bearing_cd(void) const
{
	return _L1.nav_bearing_cd();
}

int32_t AP_L2_Control::bearing_error_cd(void) const
{
	return _L1.bearing_error_cd();
}

int32_t AP_L2_Control::target_bearing_cd(void) const
{
	return _target_bearing_cd;
}

float AP_L2_Control::turn_distance(float wp_radius) const
{
	return _L1.turn_distance(wp_radius);
}


float AP_L2_Control::turn_distance(float wp_radius, float turn_angle) const
{
	return _L1.turn_distance(wp_radius, turn_angle);
}

bool AP_L2_Control::reached_loiter_target(void)
{
	return _WPcircle;
}

float AP_L2_Control::crosstrack_error(void) const
{
	return _L1.crosstrack_error();
}

void AP_L2_Control::update_level_flight(void)
{	
	_L1.update_level_flight();
}

void AP_L2_Control::update_heading_hold(int32_t navigation_heading_cd)
{
	_L1.update_heading_hold(navigation_heading_cd);
}


