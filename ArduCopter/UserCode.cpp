/// -*- tab-width: 4; Mode: C++; c-basic-offset: 4; indent-tabs-mode: nil -*-


/*///////////////////////////////////////////////////////////////////////////////////
 Last Edited: 2015/10/22
 Version:     V2.02

 Functionality:
 Custom LED code
 Custom battery consumption code - to initiate warning and then RTL to allow a return to home with sufficient battery
 IR-Lock precision landing during RTL and LAND. Also allow RTL land pause and resume if lose and then gain IR-Lock fix

 Version History:
 v2.02 Fix calculations for distance to moveable home (remove RTM location) and use AHRS home always
 v2.01 Add custom Battery failsafe code and GCS
 v2.00 Add custom LED (only) code to AC3.3
 v1.06 Add parameter IRLOCK_TIMEOUT to allow changing the IR-Lock fix timeout
       Add RTL auto pause feature. Which means if landing in RTL and we lose IR-Lock fix it will switch to GUIDED. If we then regain IR-Lock fix then it will switch back to RTL
       and just straight to the landing stage. If you switch out of GUIDED this auto pause will be disabled.
 V1.05 V5 Change message sevrity
 V1.04 V4 Add messages for regaining IR-Lock fix and allow landing to continue in LAND
 V1.03 Add Return to me (changing RTL location). Change to guided if RTL and no IR-Lock, allow land to always continue. Issue audible messages when IR-Lock acquired and lost
 		 during landing phases.
 V1.02 Change LED and battery consumption code to use the new RTL updated home location 
 V1.01 Same as V1.0 but upgraded to IR-Lock V3.2.1 - for basic testing before adding more code 
 V1.00 Custom LED and battery consumption code

 Notes:
 Add more comments to start of two main functions
 Are temp_1...3 variables needed


///////////////////////////////////////////////////////////////////////////////////
// Functions:

// Turn TTL or PWM outputs on corresponding to correct direction to Home location
void Copter::custom_led()
// Calculate remaining flight time available to return time based on current usage. Trigger 'Bingo' RTL failsafe.
void Copter::batt_consumption()
// Calculate distance and bearing from vehicle to home location
void Copter::calc_rtm_distance_bearing()
// Used to set and disable IR-Lock RTL pause feature from other functions
void Copter::set_irlock_rtl_pause(bool pause_state)
// Check whether we can automatically switch back to RTL mode after attempting to land with IR-Lock and we lose fix so it switches to GUIDED
void Copter::check_rtl_resume()
// Used to set and disable IR-Lock LAND pause feature from other functions
void Copter::set_irlock_land_pause(bool pause_state)
// Check whether we can automatically switch back to LAND mode after attempting to land with IR-Lock and we lose fix so it switches to GUIDED
void Copter::check_land_resume()
// Debug IR-Lock state messages
void Copter::debug_irlock()

///////////////////////////////////////////////////////////////////////////////////*/
// Notes on Parameter Setup
/*

Battery Consumption parameters:
BATT_CAPACITY	=	3300		Capacity of battery (mah)
WPNAV_SPEED		=	500			Horizontal travel speed in automatic modes (cm/s)
WPNAV_SPEED_DN	=	150			Descent speed during automatic modes (cm/s)
LAND_SPEED		=	50			Descent speed for final stage of landing (cm/s)
BATT_AVG_XY_MA	=	0			Average battery consumption when flying horizontal (ma/s) 0 = Auto-calculate
BATT_AVG_Z_MA	=	0			Average battery consumption when flying vertical (ma/s)  0 = use BATT_AVG_XY_MA
BATT_HOME_WARN	=	120			Time before battery consumption failsafe ‘bingo’ is going to trigger (seconds). 
								This will generate a warning message on GCS to indicate you have X seconds left before ‘bingo’ will trigger. 
								Note this will only be accurate if the distance to home doesn’t change. 0 = warning message will not be generated.
BATT_HOME_FS	=	500			Setpoint to trigger battery consumption failsafe ‘bingo’. This will generate a warning message on GCS and activate RTL. 
								This value is the mah required to be left once RTL is complete (mah). 0 = failsafe will not be active.

2-Pin TTL LEDs + single PWM pin parameters:
BRD_PWM_COUNT = 4               Aux 1-4 can be used for PWM (servo) outputs, the rest are relay outputs
RC10_FUNCTION = 0      			Allow AUX2 PWM (servo) Output to be controlled automatically by code.
RELAY_PIN = 54                  Relay1 is mapped to pin 54 (Aux5)
RELAY_PIN2 = 55                 Relay2 is mapped to pin 55 (Aux6)
RELAY_PIN3 = -1                 Relay3 is not used
RELAY_PIN4 = -1                 Relay4 is not used

Outputs Names:
Chan  | PWM  |	Relay Pin
--------------------------
AUX1	RC9		pin 50
AUX2	RC10	pin 52
AUX3	RC11	pin 52
AUX4	RC12	pin 53
AUX5	RC13	pin 54
AUX6	RC14	pin 55
*/

#include "Copter.h"

/////////////////////////////////////////////////////////////////////////////////////
// Defines/Variables for all custom code
static bool last_irlock_state;			// Track IR-Lock fix state so we can debug when it is on and off
static uint32_t irlock_rtl_pause_start_time = 0;

/////////////////////////////////////////////////////////////////////////////////////
// Defines/Variables for custom battery consumption firmware

//#define BATT_DEBUG					// Enable debug messages
#define BATT_DEBUG_COUNT 2000		// 2000 ~= 40s (in 50hz loop)

static uint32_t takeoff_time;			// Time at takeoff (used to calculate average ma / second)
static int32_t time_to_home_xy;			// Time (s) to fly home at average speed
static int32_t time_to_fs;				// Time until failsafe occurs
static float airborne_time;			// Time in air since takeoff (seconds)

static float takeoff_mah;				// Mah used at takeoff (used to calculate average ma / second)
static float avg_xy_ma_per_sec;			// Average mA used per second while flying
static float avg_z_ma_per_sec;			// Average mA used per second while landing
static float required_ma_to_home;		// Required mA to reach home
static float batt_cap_remaining;		// Remaining capacity in pack

static bool takeoff_flag;				// Used to detect when taking off
static bool fs_batt_home_warn_flag;
static bool fs_batt_home_flag;

/////////////////////////////////////////////////////////////////////////////////////
// Defines/Variables for custom LED firmware

//#define LED_QUADRANT_DEBUG		// Enable debug messages every 3.3s
//#define LED_QUADRANT_STATUS		// Enable debug messages every 3.3s
//#define LED_QUADRANT_YAWTEST	// Test relay outputs using UAV yaw only (this would simulate home postion at North pole)

// NOTE: We only use the two TTL output and pwm variants now
//#define LED_QUADRANT_1			// Use 1 PWM servo output to indicate direction to home from UAV bearing
#define LED_QUADRANT_2			// Use two TTL outputs to indicate direction to home from UAV bearing
// NOT USED NOW #define LED_QUADRANT_4			// Use four TTL outputs to indicate direction to home from UAV bearing

#define LED_SERVO_RC 10			// Servo output to use i.e 9 = AUX1 ... 14 = AUX6 (AUX5 or AUX6 aren't working).
// Make sure the corresponding RC*_FUNCTION = 0 and BRD_PWM_COUNT parameter is correct depending which servo output you are using
// ServoRelayEvents.do_set_servo(channel, period_us) - For some reason the below PWM values are not what Evangelos is getting
#define LED_SERVO_NORTH 1100	// PWM value for North Quadrant
#define LED_SERVO_EAST 1200		// PWM value for East Quadrant
#define LED_SERVO_SOUTH 1300	// PWM value for South Quadrant
#define LED_SERVO_WEST 1400		// PWM value for West Quadrant

// Variables for custom LED firmware
static int32_t update_count;                // Used to create timing for Bingo debug messages
static int32_t update_count_led;			// Used to create timing for LED debug messages
static int32_t last_bearing_quadrant;		// Used to determine if the bearing quandrant has changed

///////////////////////////////////////////////////////////////////////////////////////


#ifdef USERHOOK_INIT
void Copter::userhook_init()
{
    // put your initialisation code here
    // this will be called once at start-up

	// Initiase variables here
    //rtm_allow = false;            
	rtm_update_location = false;  
	//rtm_distance;                 
	//rtm_bearing;                  
	irlock_fix_last = false;      
	irlock_rtl_pause = false;     
	irlock_land_pause = false; 

	land_repo_active_last = false;

}
#endif

#ifdef USERHOOK_FASTLOOP
void Copter::userhook_FastLoop()
{
    // put your 100Hz code here
}
#endif

#ifdef USERHOOK_50HZLOOP
void Copter::userhook_50Hz()
{
    // put your 50Hz code here

	// Check whether we can resume RTL or LAND IR-Lock land from guided
	check_rtl_resume();
	check_land_resume();

	// Retrieve state of IRLock device
	irlock_blob_detected = precland.have_lock();

	debug_irlock();

	// Update distance and bearing to Return-to-me location
	calc_rtm_distance_bearing();

	batt_consumption();
	custom_led();
}
#endif

#ifdef USERHOOK_MEDIUMLOOP
void Copter::userhook_MediumLoop()
{
    // put your 10Hz code here
}
#endif

#ifdef USERHOOK_SLOWLOOP
void Copter::userhook_SlowLoop()
{
    // put your 3.3Hz code here
}
#endif

#ifdef USERHOOK_SUPERSLOWLOOP
void Copter::userhook_SuperSlowLoop()
{
    // put your 1Hz code here
}
#endif

// Turn TTL or PWM outputs on corresponding to correct direction to Home location
void Copter::custom_led()
{
	   
#ifdef LED_QUADRANT_YAWTEST
	// For testing use only the current yaw angle to drive the outputs
	int32_t mav_to_rtm_bearing = (360-ahrs.yaw_sensor)/100;
#else
	// Calculate the bearing to the home location from the current location, relative to the UAV yaw
	int32_t mav_to_rtm_bearing = (rtm_bearing-ahrs.yaw_sensor)/100;
#endif  // LED_QUADRANT_YAWTEST

	// Normalize in case over rollover to negative bearing
	if (mav_to_rtm_bearing < 0)
		mav_to_rtm_bearing = mav_to_rtm_bearing + 360;
	
	// MAV bearing to home in NORTH quadrant
	if ((mav_to_rtm_bearing >= 315 or mav_to_rtm_bearing < 45) and last_bearing_quadrant != 1)
	{
#ifdef LED_QUADRANT_STATUS
		gcs_send_text_fmt(PSTR("Activating North LED. Bearing to home: %d"), mav_to_rtm_bearing);
#endif	// LED_QUADRANT_STATUS

#ifdef LED_QUADRANT_4	
		ServoRelayEvents.do_set_relay(0, 1);	// Relay1
		ServoRelayEvents.do_set_relay(1, 0);	// Relay2
		ServoRelayEvents.do_set_relay(2, 0);	// Relay3
		ServoRelayEvents.do_set_relay(3, 0);	// Relay4
#endif  // LED_QUADRANT_4

#ifdef LED_QUADRANT_2	
		ServoRelayEvents.do_set_relay(0, 1);	// Relay1
		ServoRelayEvents.do_set_relay(1, 1);	// Relay2
#endif  // LED_QUADRANT_2

#ifdef LED_QUADRANT_1	
		ServoRelayEvents.do_set_servo(LED_SERVO_RC, LED_SERVO_NORTH);	// Servo1 (RC9 = AUX1)
#endif  // LED_QUADRANT_1

		last_bearing_quadrant = 1;
	}
	// MAV bearing to home in EAST quadrant
	if (mav_to_rtm_bearing >= 45 and mav_to_rtm_bearing < 135 and last_bearing_quadrant != 2)
	{
#ifdef LED_QUADRANT_STATUS
		gcs_send_text_fmt(PSTR("Activating East LED. Bearing to home: %d"), mav_to_rtm_bearing);
#endif // LED_QUADRANT_STATUS

#ifdef LED_QUADRANT_4
		ServoRelayEvents.do_set_relay(0, 0);	// Relay1
		ServoRelayEvents.do_set_relay(1, 1);	// Relay2
		ServoRelayEvents.do_set_relay(2, 0);	// Relay3
		ServoRelayEvents.do_set_relay(3, 0);	// Relay4
#endif  // LED_QUADRANT_4

#ifdef LED_QUADRANT_2	
		ServoRelayEvents.do_set_relay(0, 1);	// Relay1
		ServoRelayEvents.do_set_relay(1, 0);	// Relay2
#endif  // LED_QUADRANT_2

#ifdef LED_QUADRANT_1	
		ServoRelayEvents.do_set_servo(LED_SERVO_RC, LED_SERVO_EAST);	// Servo1 (RC9 = AUX1)
#endif  // LED_QUADRANT_1
		last_bearing_quadrant = 2;
	}
	// MAV bearing to home in SOUTH quadrant
	if (mav_to_rtm_bearing >= 135 and mav_to_rtm_bearing < 225 and last_bearing_quadrant != 3)
	{
#ifdef LED_QUADRANT_STATUS
		gcs_send_text_fmt(PSTR("Activating South LED. Bearing to home: %d"), mav_to_rtm_bearing);
#endif // LED_QUADRANT_STATUS

#ifdef LED_QUADRANT_4
		ServoRelayEvents.do_set_relay(0, 0);	// Relay1
		ServoRelayEvents.do_set_relay(1, 0);	// Relay2
		ServoRelayEvents.do_set_relay(2, 1);	// Relay3
		ServoRelayEvents.do_set_relay(3, 0);	// Relay4
#endif  // LED_QUADRANT_4

#ifdef LED_QUADRANT_2	
		ServoRelayEvents.do_set_relay(0, 0);	// Relay1
		ServoRelayEvents.do_set_relay(1, 1);	// Relay2
#endif  // LED_QUADRANT_2

#ifdef LED_QUADRANT_1	
		ServoRelayEvents.do_set_servo(LED_SERVO_RC, LED_SERVO_SOUTH);	// Servo1 (RC9 = AUX1)
#endif  // LED_QUADRANT_1

		last_bearing_quadrant = 3;
	}
	// MAV bearing to home in West quadrant
	if (mav_to_rtm_bearing >= 225 and mav_to_rtm_bearing < 315 and last_bearing_quadrant != 4)
	{
#ifdef LED_QUADRANT_STATUS
		gcs_send_text_fmt(PSTR("Activating West LED. Bearing to home: %d"), mav_to_rtm_bearing);
#endif // LED_QUADRANT_STATUS

#ifdef LED_QUADRANT_4
		ServoRelayEvents.do_set_relay(0, 0);	// Relay1
		ServoRelayEvents.do_set_relay(1, 0);	// Relay2
		ServoRelayEvents.do_set_relay(2, 0);	// Relay3
		ServoRelayEvents.do_set_relay(3, 1);	// Relay4
#endif  // LED_QUADRANT_4

#ifdef LED_QUADRANT_2	
		ServoRelayEvents.do_set_relay(0, 0);	// Relay1
		ServoRelayEvents.do_set_relay(1, 0);	// Relay2
#endif  // LED_QUADRANT_2

#ifdef LED_QUADRANT_1	
		ServoRelayEvents.do_set_servo(LED_SERVO_RC, LED_SERVO_WEST);	// Servo1 (RC9 = AUX1)
#endif  // LED_QUADRANT_1

		last_bearing_quadrant = 4;
	}

#ifdef LED_QUADRANT_DEBUG
	// For debugging output the calculated values every ~ 3.3s
	if (update_count_led > 500)
	{
		gcs_send_text_fmt(PSTR("LED_QUAD hb: %d yaw: %d mav_home: %d"), rtm_bearing/100, ahrs.yaw_sensor/100, mav_to_rtm_bearing);
		update_count_led = 0;
	}
	update_count_led = update_count_led + 1;
#endif  // LED_QUADRANT_DEBUG

}

////////////////////////////////////////////////
//
// Battery Consumption Code
//
// This tracks the current usage since takeoff and calculates an average ma per sec
// We then calculate how long (seconds) it will take to reach home from current location, and thus how many ma it will take
// We can then predict when we should return home to allow a certain ma of battery left once landed at home

// Improvements:
// Could give warning like 'Return home now to retain 30% battery'
// Log battery capacity events to log file
// If land we should reset takeoff flag so we can calculate fresh again
void Copter::batt_consumption()
{
	uint32_t tnow = hal.scheduler->millis();

	// Record ma used at takeoff
	if ((current_loc.alt > 200) and !takeoff_flag)
	{
		takeoff_mah = battery.current_total_mah();
		takeoff_time = tnow;	
		
		#ifdef BATT_DEBUG
			gcs_send_text_fmt(PSTR("Takeoff ma: %0f"), takeoff_mah);
		#endif  // BATT_DEBUG

		takeoff_flag = true;		
	}

	// If landed then we should reset the takeoff_flag to re-calculate average ma/s
	if (ap.land_complete and takeoff_flag)
	{
		fs_batt_home_warn_flag = false;
		fs_batt_home_flag = false;
		takeoff_flag = false;
		//gcs_send_text_fmt(PSTR("Landed; reset takeoff_flag"));
	}

	// capacity_pack_total comes from parameter BATT_CAPACITY
	batt_cap_remaining = battery.capacity_pack_total() - battery.current_total_mah();

	airborne_time = (float(tnow) - float(takeoff_time))/1000;

	// Only calculate average ma/s if we have taken off
	if (takeoff_flag)
	{
		// If parameter BATT_AVG_XY_MA = 0 then automatically calculate the average ma/s since takeoff
		if (g.batt_avg_xy_ma == 0.0f)
		{
			if (airborne_time > 1.0f)	// Protection against / by 0
			{
				avg_xy_ma_per_sec = (battery.current_total_mah() - takeoff_mah) / airborne_time;
				avg_xy_ma_per_sec = (battery.current_total_mah() - takeoff_mah) / airborne_time;
			}
		}
		else
			avg_xy_ma_per_sec = g.batt_avg_xy_ma;

		// If parameter BATT_AVG_Z_MA = 0 then use BATT_AVG_XY_MA as everage ma/s while landing
		if (g.batt_avg_z_ma == 0.0f)
			avg_z_ma_per_sec = avg_xy_ma_per_sec;
		else
			avg_z_ma_per_sec = g.batt_avg_z_ma;
	}
	else
		avg_xy_ma_per_sec = 0.0f;

	/////////////////////////////////////////////////////////////////
	// Calculate ma required to return to me (moveable home) from current position
	//

	if (avg_xy_ma_per_sec > 0.0f) {
		// ma to cover horizontal flight when flying to home position
		// Plus 10s delays in RTL and changing direction etc.
		if (wp_nav.get_speed_xy() != 0)		// Protection against / by 0
			time_to_home_xy = (rtm_distance) / wp_nav.get_speed_xy();
			//time_to_home_xy = (rtm_distance/100) / batt_avg_xy_spd;
			
		required_ma_to_home = (time_to_home_xy + 5) * avg_xy_ma_per_sec;
		
		// NOTE:
		// When landing the vertical speed is defined by wp_nav.get_speed_down() (parameter WPNAV_SPEED_DN)
		// when altitude is above LAND_START_ALT (defined as 10m in config.h). 
		// When altitude is belove LAND_START_ALT then the vertical speed is defined by g.land_speed (parameter LAND_SPEED)
		
		// ma to cover vertical flight when landing (above LAND_START_ALT = 10m)
		if ((wp_nav.get_speed_down() != 0) and (current_loc.alt > LAND_START_ALT))		// Protection against / by 0
			required_ma_to_home += ((current_loc.alt - LAND_START_ALT) / wp_nav.get_speed_down()) * avg_z_ma_per_sec;

		// ma to cover vertical flight when landing (below LAND_START_ALT = 10m)
		if ((g.land_speed != 0) and (current_loc.alt >= LAND_START_ALT))		// Protection against / by 0
			required_ma_to_home += (LAND_START_ALT / (float)g.land_speed) * avg_z_ma_per_sec;
		
		// Calculate time until failsafe will occur
		time_to_fs = ((batt_cap_remaining - required_ma_to_home) - g.batt_home_fs) / avg_xy_ma_per_sec;
	
	}
	else {
		time_to_fs = 0;
		required_ma_to_home = 0.0f;
	}

	// Warning - Generate warning message
	// Do not generate warning if parameter BATT_HOME_WARN = 0
	if (time_to_fs != 0)
	{
		if ((g.batt_home_warn > 0) and ((float)time_to_fs < g.batt_home_warn) and !fs_batt_home_warn_flag)
		{
			gcs_send_text_P(SEVERITY_HIGH,PSTR("BATT: Time warning!"));
			gcs_send_text_fmt(PSTR("Batt: %ds remaining!"), time_to_fs);

			fs_batt_home_warn_flag = true;
		}
	}
	
	/*
	if (((batt_cap_remaining - required_ma_to_home) < g.batt_home_warn) and !fs_batt_home_warn_flag)
	{
		gcs_send_text_fmt(PSTR("Batt Home Warning - used: %0f left: %0f"), battery.current_total_mah(), batt_cap_remaining);
		fs_batt_home_warn_flag = true;
	}
	*/

	// Alarm - Generate alarm message and trigger RTL
	// Do not generate alarm if parameter BATT_HOME_FS = 0
	if (required_ma_to_home != 0.0f) {
		if ((g.batt_home_fs > 0.0f) and ((batt_cap_remaining - required_ma_to_home) < g.batt_home_fs) and !fs_batt_home_flag)
		{
			gcs_send_text_fmt(PSTR("Batt Home Alarm (RTL) - used: %0f left: %0f"), battery.current_total_mah(), batt_cap_remaining);
			
			// Only implement failsafe if another failsafe not already active
		    if (!failsafe.battery) {
		        // Initiate a RTL or Land
				if (!set_mode(RTL)) {
		        	set_mode_land_with_pause();
		        }
				// set the low battery flag
			    set_failsafe_battery(true);

			    // warn the ground station and log to dataflash
			    gcs_send_text_P(SEVERITY_HIGH,PSTR("BATT: Low Battery! RTL"));
			    Log_Write_Error(ERROR_SUBSYSTEM_FAILSAFE_BATT, 9);
				
				fs_batt_home_flag = true;
		    }
		}
	}


#ifdef BATT_DEBUG
	// For debugging, output the calculated values every ~ 40s
	if (update_count == 1)
	{
		//gcs_send_text_fmt(PSTR("Batt used: %.0fmah, %d%% - dist_home: %.0fm"), battery.current_total_mah(), battery.capacity_remaining_pct(), rtm_distance/100);
		gcs_send_text_fmt(PSTR("rtm_bearing: %.0f - dist_home: %.0fm"), rtm_bearing, rtm_distance/100);
	}

	if (update_count == 100)
		gcs_send_text_fmt(PSTR("Avg %.1fma/s, time to home: %ds, ma to home: %.0f"), avg_xy_ma_per_sec, time_to_home_xy, required_ma_to_home);
		
	if (update_count == 200)
		gcs_send_text_fmt(PSTR("Batt remaining: %.0f of %.0fmah, %.0f, ttf: %ds"), batt_cap_remaining, battery.capacity_pack_total(), (batt_cap_remaining - required_ma_to_home),time_to_fs);

	update_count += 1;
	
	// Reset counter to zero
	if (update_count > BATT_DEBUG_COUNT)
		update_count = 0;

#endif  // BATT_DEBUG

}

// Calculate distance and bearing from vehicle to home location
void Copter::calc_rtm_distance_bearing()
{
    // get current location
    Vector3f curr = inertial_nav.get_position();
	
	// We DON'T USE THIS ANYMORE in AC3.3 as we are able to set home location
#if 0    
	// Set destination as default original takeoff location
    //Vector3f destination = Vector3f(0,0,0);
    
    // Only change to Return-to-me location if a valid new home position has been received
    //if (rtm_allow)
    //    destination = pv_location_to_vector(rtm_loc);
#else    
    // New for AC 3.3
    Vector3f destination = pv_location_to_vector(ahrs.get_home());
#endif

	// Calculate distance to Return-to-me location
	rtm_distance = pythagorous2(destination.x-curr.x,destination.y-curr.y);

	// Get bearing in centi-degrees
	rtm_bearing = pv_get_bearing_cd(curr, destination);

}

// Used to set and disable IR-Lock RTL pause feature from other functions
void Copter::set_irlock_rtl_pause(bool pause_state)
{
	if (pause_state)
	{
		irlock_rtl_pause_start_time = hal.scheduler->millis();
		irlock_rtl_pause = true;
		//gcs_send_text_fmt(PSTR("GG RTL Pause Start Time: %d"), irlock_rtl_pause_start_time);
	}
	else
		irlock_rtl_pause = false;
}

// Check whether we can automatically switch back to RTL mode after attempting to land with IR-Lock and we lose fix so it switches to GUIDED
void Copter::check_rtl_resume()
{
	// get the current time
    //uint32_t now = hal.scheduler->millis();

	if (!irlock_rtl_pause)
		return;

	// Turn off pause if we are not in GUIDED mode
	if (control_mode != GUIDED)
	{
		irlock_rtl_pause = false;
		return;
	}	

	// Disable IR-Lock RTL resume after 60seconds or if distance to home is greater than 10m
	//if (((now - irlock_rtl_pause_start_time) > IRLOCK_PAUSE_TIMEOUT) || rtm_distance >= IRLOCK_PAUSE_MAX_DIST)
	/* Disable this timeout feature as Evangelos doesn't require it. NOTE: This hasn't been fully simulated yet
	if ((now - irlock_rtl_pause_start_time) > IRLOCK_PAUSE_TIMEOUT)
	{
		uint32_t diffTime = now - irlock_rtl_pause_start_time;
		irlock_rtl_pause = false;
		gcs_send_text_fmt(PSTR("GG Disable RTL resume. Time: %d"), diffTime);
	}

	if (rtm_distance >= IRLOCK_PAUSE_MAX_DIST)
	{
		irlock_rtl_pause = false;
		gcs_send_text_fmt(PSTR("GG Disable RTL resume. Dist %f"), rtm_distance);
	}
	*/
	

	// Return if IR-Lock is not enabled or we don't have a IR-Lock fix
	if ((g.irlock_alt == -1.0f) || (irlock_blob_detected != true))
    	return;

	if (irlock_rtl_pause)
	{
		if (set_mode(RTL))
        {
		    gcs_send_text_P(SEVERITY_HIGH, PSTR("IR-Lock fix acquired. Resume precision landing"));
        	//irlock_rtl_pause = false;
        }
	}
}

// Used to set and disable IR-Lock LAND pause feature from other functions
void Copter::set_irlock_land_pause(bool pause_state)
{
	if (pause_state)
	{
		//irlock_land_pause_start_time = hal.scheduler->millis();
		irlock_land_pause = true;
		//gcs_send_text_fmt(PSTR("GG Land Pause Start Time: %d"), irlock_rtl_pause_start_time);
	}
	else
		irlock_land_pause = false;
}

// Check whether we can automatically switch back to LAND mode after attempting to land with IR-Lock and we lose fix so it switches to GUIDED
void Copter::check_land_resume()
{
	// get the current time
    //uint32_t now = hal.scheduler->millis();

	if (!irlock_land_pause)
		return;

	// Turn off pause if we are not in GUIDED mode
	if (control_mode != GUIDED)
	{
		irlock_land_pause = false;
		return;
	}	

	// Return if IR-Lock is not enabled or we don't have a IR-Lock fix
	if ((g.irlock_alt == -1.0f) || (irlock_blob_detected != true))
    	return;

	if (irlock_land_pause)
	{
		if (set_mode(LAND))
        {
		    gcs_send_text_P(SEVERITY_HIGH, PSTR("IR-Lock fix acquired. Resume precision landing"));
        	irlock_land_pause = false;
        }
	}
}

void Copter::debug_irlock()
{
    // IR-Lock Fix Aquired
    if (!last_irlock_state && irlock_blob_detected)
    {
        Log_Write_Error(50, 1);
        gcs_send_text_fmt(PSTR("IR-Lock Fix Aquired"));
    }
    
    // IR-Lock Fix Lost
    if (last_irlock_state && !irlock_blob_detected)
    {
        Log_Write_Error(50, 2);
    	gcs_send_text_fmt(PSTR("IR-Lock Fix Lost"));
    }
    
    last_irlock_state = irlock_blob_detected;

    // Full debug
#if 0
    if(precland.get_sim())
    	gcs_send_text_fmt(PSTR("IR-Lock sim on"));
    else
    	gcs_send_text_fmt(PSTR("IR-Lock sim off"));

	if(precland.have_lock())
    	gcs_send_text_fmt(PSTR("IR-Lock lock on"));
    else
    	gcs_send_text_fmt(PSTR("IR-Lock lock off"));
#endif

    // Debug irlock loiter shift
#if 0
    static uint32_t debug_irlock_shift_counter = 0;

    debug_irlock_shift_counter++;

    //if (debug_irlock_shift_counter > 200 && irlock_blob_detected)
    if (debug_irlock_shift_counter > 200)
    {
    	//Vector2f irlock_ef = precland.last_bf_angle_to_target();
		//gcs_send_text_fmt(PSTR("GG earth frame: %f, %f"), irlock_ef.x, irlock_ef.y);	    	
		Vector3f irlock_target_offset = precland.last_target_pos_offset();
		gcs_send_text_fmt(PSTR("GG irlock_target_offset: %f, %f, %f"), irlock_target_offset.x, irlock_target_offset.y, irlock_target_offset.z);
    	
    	//gcs_send_text_fmt(PSTR("GG irlock update: %d, num_blocks: %d"), precland.get_last_update(), precland.get_num_blocks());	    	
		//gcs_send_text_fmt(PSTR("GG irlock counters: %d, %d, %d"), precland.get_counter1(), precland.get_counter2(), precland.get_counter3());	    	

    	//gcs_send_text_fmt(PSTR("GG shift: %f, %f, %f"), irlock_target_shift.x, irlock_target_shift.y, irlock_target_shift.z);
    	debug_irlock_shift_counter = 0;
    }
#endif
    	
}