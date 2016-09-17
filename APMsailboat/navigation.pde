// -*- tab-width: 4; Mode: C++; c-basic-offset: 4; indent-tabs-mode: nil -*-

//****************************************************************
// Function that will calculate the desired direction to fly and distance
//****************************************************************
static void navigate()
{
	// do not navigate with corrupt data
	// ---------------------------------
	if (!have_position) {
		return;
	}

	if ((next_WP.lat == 0)||(home_is_set==false)){
		return;
	}

	// waypoint distance from rover
	// ----------------------------
	wp_distance = get_distance(current_loc, next_WP);

	if (wp_distance < 0){
		gcs_send_text_P(SEVERITY_HIGH,PSTR("<navigate> WP error - distance < 0"));
		return;
	}

	// control mode specific updates to nav_bearing
	// --------------------------------------------
	update_navigation();
}

static void update_loiter()
{
    nav_controller->update_loiter(next_WP, 5, 0); //5 meter diameter circle
}

void reached_waypoint()
{       

}

