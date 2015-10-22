/// -*- tab-width: 4; Mode: C++; c-basic-offset: 4; indent-tabs-mode: nil -*-

#include "Tracker.h"

void Tracker::user_setup()
{
	led_init();
}


// GG Added this loop for our custom code
void Tracker::user_code()
{
    led_code();
}

// Initialise GPIO and PWM outputs
void Tracker::led_init()
{
	send_custom_version();

#ifdef GCS_LED_ENABLE_TTL
	// Initialise GPIO as outputs
	hal.gpio->pinMode(GPIO_GCS_TTL1, HAL_GPIO_OUTPUT);
	hal.gpio->pinMode(GPIO_GCS_TTL2, HAL_GPIO_OUTPUT);
	hal.gpio->write(GPIO_GCS_TTL1, 0);
	hal.gpio->write(GPIO_GCS_TTL2, 0);
#if (GCS_LED_NUM_OUTPUTS >= 3)
	hal.gpio->pinMode(GPIO_GCS_TTL3, HAL_GPIO_OUTPUT);
	hal.gpio->write(GPIO_GCS_TTL3, 0);
#endif // GCS_LED_NUM_OUTPUTS
#endif // GCS_LED_ENABLE_TTL

#ifdef GCS_LED_ENABLE_PWM
	hal.rcout->enable_ch(PWM_CH_OUT);
	hal.rcout->write(PWM_CH_OUT, 1500);
	//gg_debug_usb_msg(PSTR("[led_init] Enabled PWM output %d"), PWM_CH_OUT);
	// 500 = 0.9ms
#endif // GCS_LED_ENABLE_PWM

}

/**
    GG Custom Code for LED on APM Clone
*/
void Tracker::led_code()
{
    // calculate the bearing to the vehicle
#ifndef LED_DEBUG_SIM
    int32_t bearing  = get_bearing_cd(current_loc, vehicle.location);  
    float distance = get_distance(current_loc, vehicle.location);
    float pitch_f    = degrees(atan2f(nav_status.altitude_difference, distance));
#else
    int32_t bearing  = 1000;  // 10.00 degrees
    float distance = 166.2f;
    float pitch_f    = 1.5f;    
#endif  // LED_DEBUG_SIM

    int32_t pitch = pitch_f * 100;

    // Get the base board yaw and pitch, adjusting with trim parameter
    int32_t base_yaw = wrap_360_cd(ahrs.yaw_sensor+(g.yaw_trim*100));// * 0.01f;
    int32_t base_pitch = constrain_int32((ahrs.pitch_sensor+(g.pitch_trim*100)), -9000, 9000);
    int32_t base_roll = constrain_int32((ahrs.roll_sensor+(g.pitch_trim*100)), -9000, 9000);

    // Calculate relative bearing from APM Clone to Vehicle in centi-degrees
    int32_t rel_bearing = (bearing - base_yaw);    

    // Normalize in case over rollover to negative bearing
    if (rel_bearing < 0)
        rel_bearing = rel_bearing + 36000;

    // Calculate relative pitch from APM Clone to Vehicle
    int32_t rel_pitch;

/////////////////////////////////////////////////////
    // Old method only uses either roll or pitch angle to determine relative pitch to vehicle
#if 0
    // Depending on relative bearing to vehicle we will use pitch or roll
    if ((rel_bearing > 4500 && rel_bearing < 13500) || (rel_bearing > 22500 && rel_bearing < 31500))
    {
        rel_pitch = (pitch + abs(base_roll));
        if (g.msg_debug == 2) 
     		if (debug_count == 15)
     			gg_debug_usb_msg(PSTR("Rel_pitch roll: %d"), rel_pitch);
     			
        
    }
    else
    {
        rel_pitch = (pitch + abs(base_pitch));
        if (g.msg_debug == 2)
        	if (debug_count == 15) 
        		gg_debug_usb_msg(PSTR("Rel_pitch pitch: %d"), rel_pitch);
        
    }
#endif

    // New method is more accurate
    float e2t_tilt, e2t_pan;
    float b2t_tilt, b2t_pan;
    float roll;

    // target heading / bearing calculation 
    e2t_pan = ToRad((bearing/100));  
    e2t_tilt = ToRad(pitch_f);

    Matrix3f m;
    m.from_euler (ahrs.roll, ahrs.pitch+ToRad(g.pitch_trim), ahrs.yaw+ToRad(g.yaw_trim));
    
    m.transpose();                                               // traspose matrix
    Matrix3f earth;                                              // define matrix to store target rotation  
    earth.from_euler (0, e2t_tilt, e2t_pan);                     // get earth-to-target rotation matrix (roll = 0)
    m = m * earth;                                               // multiply matrices to obtain base-to-target rotation matrix 
    m.to_euler(&roll, &b2t_tilt, &b2t_pan);                      // then revert to euler angles

    rel_pitch = ToDeg(b2t_tilt) * 100;

    // Set GPIO outputs as per matrix
    if (rel_pitch > 4500)
#ifdef GCS_LED_ENABLE_8_SECTORS
    	// Only set different outputs for pitch > 45 degrees if using 8 LED version
        led_above_45(rel_bearing);
#else
    	led_below_45(rel_bearing);
#endif // GCS_LED_ENABLE_8_SECTORS
    else
        led_below_45(rel_bearing);

    // Only display debug messages if parameter msg_debug is 1
    if (g.msg_debug.get() >= 1) {
        if (debug_count == 1)
            gg_debug_usb_msg(PSTR("yaw: %d, rel_bearing: %d, rel_tilt: %d"), (base_yaw/100), (rel_bearing/100), (rel_pitch/100));

        //if (debug_count == 5)
        	//gg_debug_usb_msg(PSTR("pitch: %d, b2t_tilt: %f"), (base_pitch/100), b2t_tilt);

        //    gg_debug_usb_msg(PSTR("rel_bearing: %d"), (rel_bearing/100));

        //if (debug_count == 10)
        //    gg_debug_usb_msg(PSTR("rel_tilt: %d"), (rel_pitch/100));
    }

    debug_count++;
    
    // Reset counter to zero
    if (debug_count > LED_DEBUG_COUNT)
        debug_count = 0;

  
}

/**
    Set PWM output on Ch9 = AUX1
    1000 = 1.0ms
    2000 = 2.0ms
**/
void Tracker::set_led_pwm(int16_t pwm_value)
{
    // Write PWM value to RC_3
    hal.rcout->write(PWM_CH_OUT, pwm_value);
    //gg_debug_usb_msg(PSTR("[set_led_pwm] PWM output [%d] = %d"), PWM_CH_OUT, pwm_value);
}

// Set correct outputs using a matrix array
void Tracker::set_led_matrix(int dir_sector, int pitch_sector)
{
	/*
	Case:	Dir:	Pitch:
	1		1		1
	2		1		0
	3		2		1
	4		2		0
	5		2		1
	*/
#ifdef GCS_LED_ENABLE_4_SECTORS 
    int case_num = dir_sector;
#endif
#ifdef GCS_LED_ENABLE_8_SECTORS 
    int case_num = (dir_sector * 2) - pitch_sector;
#endif

	// Limit maximum value of case (so we don't go outside array)
	if (case_num > GCS_LED_NUM_CASES)
		case_num = GCS_LED_NUM_CASES;

	// Convert case number to index
	case_num = case_num - 1;

	if (g.msg_debug.get() == 4) {
        if (debug_count == 3)
            gg_debug_usb_msg(PSTR("dir_sector: %d, pitch_sector: %d, index_num: %d"), dir_sector, pitch_sector, case_num);
    }

	// Limit minimum index of case (so we don't go outside array)
	if (case_num < 0)
		case_num = 0;

	hal.gpio->write(GPIO_GCS_TTL1, gcs_led_matrix[case_num][0]);
    hal.gpio->write(GPIO_GCS_TTL2, gcs_led_matrix[case_num][1]);
#if (GCS_LED_NUM_OUTPUTS >= 3)
    hal.gpio->write(GPIO_GCS_TTL3, gcs_led_matrix[case_num][2]);
#endif

}

void Tracker::set_led_matrix_case(int case_num)
{
	// Limit maximum value of case (so we don't go outside array)
	if (case_num > GCS_LED_NUM_CASES)
		case_num = GCS_LED_NUM_CASES;

	// Convert case number to index
	case_num = case_num - 1;

	// Limit minimum index of case (so we don't go outside array)
	if (case_num < 0)
		case_num = 0;

	hal.gpio->write(GPIO_GCS_TTL1, gcs_led_matrix[case_num][0]);
    hal.gpio->write(GPIO_GCS_TTL2, gcs_led_matrix[case_num][1]);
#if (GCS_LED_NUM_OUTPUTS >= 3)
    hal.gpio->write(GPIO_GCS_TTL3, gcs_led_matrix[case_num][2]);
#endif

}

/**
    Set LED matrix for relative angle when pitch < 45 degrees
    rel_bearing = centi-degrees
 */
void Tracker::led_below_45(int32_t rel_bearing)
{
    /*
    Case 1: North = 315 - 45 degrees
    Case 3: East  = 45  - 135
    Case 5: South = 135 - 225
    Case 7: West  = 225 - 315
    */
    if (rel_bearing > 31500 or rel_bearing < 4500)
    {
        // Case 1: North
#ifdef GCS_LED_ENABLE_PWM
        set_led_pwm(PWM_NORTH);              // Set PWM output for LED
#endif 
#ifdef GCS_LED_ENABLE_TTL
        set_led_matrix(SECTOR_DIR_NORTH);
#endif // GCS_LED_ENABLE_TTL
    }
    else if (rel_bearing > 4500 and rel_bearing < 13500)
    {
        // Case 3: East
#ifdef GCS_LED_ENABLE_PWM
        set_led_pwm(PWM_EAST);              // Set PWM output for LED
#endif 
#ifdef GCS_LED_ENABLE_TTL
        set_led_matrix(SECTOR_DIR_EAST);
#endif // GCS_LED_ENABLE_TTL
    }
    else if (rel_bearing > 13500 and rel_bearing < 22500)
    {
        // Case 5: South
#ifdef GCS_LED_ENABLE_PWM
        set_led_pwm(PWM_SOUTH);              // Set PWM output for LED
#endif
#ifdef GCS_LED_ENABLE_TTL
        set_led_matrix(SECTOR_DIR_SOUTH);
#endif // GCS_LED_ENABLE_TTL
    }
    else if (rel_bearing > 22500 and rel_bearing < 31500)
    {
        // Case 7: West
#ifdef GCS_LED_ENABLE_PWM
        set_led_pwm(PWM_WEST);              // Set PWM output for LED
#endif 
#ifdef GCS_LED_ENABLE_TTL
        set_led_matrix(SECTOR_DIR_WEST);
#endif // GCS_LED_ENABLE_TTL
    }
}


/**
    Set LED matrix for relative angle when pitch > 45 degrees
    rel_bearing = centi-degrees
 */
#ifdef GCS_LED_ENABLE_8_SECTORS
void Tracker::led_above_45(int32_t rel_bearing)
{
    /*
    Case 2: North = 315 - 45 degrees
    Case 4: East  = 45  - 135
    Case 6: South = 135 - 225
    Case 8: West  = 225 - 315
    */
    if (rel_bearing > 31500 or rel_bearing < 4500)
    {
        // Case 2: North > 45 degrees pitch
#ifdef GCS_LED_ENABLE_PWM
        set_led_pwm(PWM_NORTH_UP);              // Set PWM output for LED
#endif 
#ifdef GCS_LED_ENABLE_TTL
        set_led_matrix(SECTOR_DIR_NORTH, SECTOR_PITCH_OVER45);
#endif // GCS_LED_ENABLE_TTL
    }
    else if (rel_bearing > 4500 and rel_bearing < 13500)
    {
        // Case 4: East > 45 degrees pitch
#ifdef GCS_LED_ENABLE_PWM
        set_led_pwm(PWM_EAST_UP);              // Set PWM output for LED
#endif 
#ifdef GCS_LED_ENABLE_TTL
        set_led_matrix(SECTOR_DIR_EAST, SECTOR_PITCH_OVER45);
#endif // GCS_LED_ENABLE_TTL
    }
    else if (rel_bearing > 13500 and rel_bearing < 22500)
    {
        // Case 6: South > 45 degrees pitch
#ifdef GCS_LED_ENABLE_PWM
        set_led_pwm(PWM_SOUTH_UP);              // Set PWM output for LED
#endif 
#ifdef GCS_LED_ENABLE_TTL
        set_led_matrix(SECTOR_DIR_SOUTH, SECTOR_PITCH_OVER45);
#endif // GCS_LED_ENABLE_TTL
    }
    else if (rel_bearing > 22500 and rel_bearing < 31500)
    {
        // Case 8: West > 45 degrees pitch
#ifdef GCS_LED_ENABLE_PWM
        set_led_pwm(PWM_WEST_UP);              // Set PWM output for LED
#endif 
#ifdef GCS_LED_ENABLE_TTL
        set_led_matrix(SECTOR_DIR_WEST, SECTOR_PITCH_OVER45);
#endif // GCS_LED_ENABLE_TTL
    }
}
#endif // GCS_LED_ENABLE_8_SECTORS

// Debug message
// This forces out USB only, so we don't send junk to Autopilot over Radio
void Tracker::gg_debug_usb_msg(const prog_char_t *fmt, ...)
{
// Plain text only or formatted for STATUSTEXT
#if 0       
    char msg[50];
    va_list arg_list;
    va_start(arg_list, fmt);
    hal.util->vsnprintf_P((char *)msg, sizeof(msg), fmt, arg_list);
    va_end(arg_list);

    hal.console->printf_P(PSTR("[gg_debug_usb_msg] %s\n"), msg);
#else
    va_list arg_list;
    //gcs[0].pending_status.severity = (uint8_t)SEVERITY_LOW;
    gcs[0].pending_status.severity = (uint8_t)MAV_SEVERITY_WARNING;
    va_start(arg_list, fmt);
    hal.util->vsnprintf_P((char *)gcs[0].pending_status.text,
            sizeof(gcs[0].pending_status.text), fmt, arg_list);
    va_end(arg_list);

    gcs[0].send_message(MSG_STATUSTEXT);        // gcs[0] is always console (which is USB if connected)
#endif

}

void Tracker::gg_debug_msg(const prog_char_t *fmt, ...)
{
    char msg[50];
    va_list arg_list;
    va_start(arg_list, fmt);
    hal.util->vsnprintf_P((char *)msg, sizeof(msg), fmt, arg_list);
    va_end(arg_list);

    gcs_send_text_fmt(PSTR("%s"), msg);
}

// Send information about this custom firmware version
// Call this when MAVLINK_MSG_ID_AUTOPILOT_VERSION_REQUEST is received in GCS_Mavlink.cpp
void Tracker::send_custom_version()
{
    char version_string[70] = THIS_CUSTOM_VERSION;



    #ifdef GCS_LED_ENABLE_4_SECTORS
        strcat(version_string, " - 4 Sectors");
    #endif
    #ifdef GCS_LED_ENABLE_8_SECTORS
        strcat(version_string, " - 8 Sectors");
	#endif
    #ifdef GCS_LED_ENABLE_TTL
        strcat(version_string, " - TTL Output");
    #endif
    #ifdef GCS_LED_ENABLE_PWM
        strcat(version_string, " - PWM Output");
    #endif

    gcs_send_text_fmt(PSTR("%s"), version_string);

}