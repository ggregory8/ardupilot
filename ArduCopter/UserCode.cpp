/// -*- tab-width: 4; Mode: C++; c-basic-offset: 4; indent-tabs-mode: nil -*-


/*///////////////////////////////////////////////////////////////////////////////////
 Last Edited: 2015/10/12
 Version:     V2.00

 Functionality:
 Custom LED code
 Custom battery consumption code - to initiate warning and then RTL to allow a return to home with sufficient battery
 IR-Lock precision landing during RTL and LAND. Also allow RTL land pause and resume if lose and then gain IR-Lock fix

 Version History:
 v2.00 Added custom LED (only) code to AC3.3
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

///////////////////////////////////////////////////////////////////////////////////*/
// Notes on Parameter Setup
/*

2-Pin TTL LEDs + single PWM pin parameters:
BRD_PWM_COUNT = 2               Aux 1-2 can be used for PWM (servo) outputs, the rest are relay outputs
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
// Defines/Variables for custom LED firmware

//#define LED_QUADRANT_DEBUG		// Enable debug messages every 3.3s
//#define LED_QUADRANT_STATUS		// Enable debug messages every 3.3s
//#define LED_QUADRANT_YAWTEST	// Test relay outputs using UAV yaw only (this would simulate home postion at North pole)

// NOTE: We only use the two TTL output and pwm variants now
//#define LED_QUADRANT_1			// Use 1 servo output to indicate direction to home from UAV bearing
#define LED_QUADRANT_2			// Use two TTL outputs to indicate direction to home from UAV bearing
// NOT USED NOW 
//#define LED_QUADRANT_4			// Use four TTL outputs to indicate direction to home from UAV bearing

#define LED_SERVO_RC 10			// Servo output to use i.e 9 = AUX1 ... 14 = AUX6 (AUX5 or AUX6 aren't working).
// Make sure the corresponding RC*_FUNCTION = 0 and BRD_PWM_COUNT parameter is correct depending which servo output you are using
// ServoRelayEvents.do_set_servo(channel, period_us) - For some reason the below PWM values are not what Evangelos is getting
#define LED_SERVO_NORTH 1100	// PWM value for North Quadrant
#define LED_SERVO_EAST 1200		// PWM value for East Quadrant
#define LED_SERVO_SOUTH 1300	// PWM value for South Quadrant
#define LED_SERVO_WEST 1400		// PWM value for West Quadrant

// Variables for custom LED firmware
static int32_t update_count_led;			// Used to create timing for debug messages
static int32_t last_bearing_quadrant;		// Used to determine if the bearing quandrant has changed

///////////////////////////////////////////////////////////////////////////////////////


#ifdef USERHOOK_INIT
void Copter::userhook_init()
{
    // put your initialisation code here
    // this will be called once at start-up
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

void Copter::custom_led()
{
	   
#ifdef LED_QUADRANT_YAWTEST
	// For testing use only the current yaw angle to drive the outputs
	int32_t mav_to_rtm_bearing = (360-ahrs.yaw_sensor)/100;
#else
	// Calculate the bearing to the home location from the current location, relative to the UAV yaw
	int32_t mav_to_rtm_bearing = (home_bearing-ahrs.yaw_sensor)/100;
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
		update_count = 0;
	}
	update_count = update_count + 1;
#endif  // LED_QUADRANT_DEBUG

}