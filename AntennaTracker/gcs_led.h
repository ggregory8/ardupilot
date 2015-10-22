// -*- tab-width: 4; Mode: C++; c-basic-offset: 4; indent-tabs-mode: nil -*-

#ifndef _GCS_LED_H
#define _GCS_LED_H

/*///////////////////////////////////////////////////////////////////////////////////

Custom GCS LED Code

Version:
0.2:
NOTE: This version has not been tested on the APM2.5 clone
Adjust to new code format using classes
Allow to run on Pixhawk as GCS base unit (instead of APM2.5 clone)

0.1:
Adjust Antenna Tracker to output a matrix LED array in indicate relative direction to vehicle (running on APM2.5 clone boart)

If parameter msg_debug
0 = No debug messages
1 = Show base_yaw, rel_bearing, rel_pitch values
2 = Show relative_pitch values (also shows if base pitch or roll is used)

Improvements:
Can use vehicle.location_estimate if we are losing packets to esitmate position


*////////////////////////////////////////////////////////////////////////////////////
#define THIS_CUSTOM_VERSION		"GCS LED V0.2"		// This gets sent to GCS when first connecting


#define LED_DEBUG               		// Display debug messages to GCS. MSG_DEBUG parameter can turn these on and off
#define LED_DEBUG_COUNT 		15    	//                    2000 ~= 40s (in 50hz loop)
//#define LED_DEBUG_SIM           		// Simulate vehicle data (bearing and pitch)

#define USER_CODE_RATE  		5     	// rate in 20ms units i.e 5 = 10Hz, 50 = 1hz, 250 = once every 5s

//#define APM25_CLONE						// Uncomment to use on APM2.5 clone hardware, other PX4 used (NOTE: This hassn't been tested in this new firmware)

/////////////////////////////////////////////////////////////////////////////////////
//
// Define which LED output version is going to be used 2 TTL outputs for 4 LEDs, or 3 TTL outputs for 8 LEDs.
// Also fine the number of cases and outputs. This is used in the set_led_matrix() function to control the outputss

#define GCS_LED_ENABLE_PWM              	  	  	// Use PWM output instead of individual TTL outputs
//#define GCS_LED_ENABLE_TTL 							// Use TTL outputs to control LED board

//#define GCS_LED_ENABLE_4_SECTORS					// Enable 4 sectors (north, south, east, west)
#define GCS_LED_ENABLE_8_SECTORS					// Enable 8 sectors (as per 4 sectors but above 45 degrees pitch and below)


#ifdef GCS_LED_ENABLE_4_SECTORS
#ifdef GCS_LED_ENABLE_8_SECTORS
	#error "Can't define GCS_LED_ENABLE_4_SECTORS and GCS_LED_ENABLE_8_SECTORS!"
#endif
#endif

#ifndef GCS_LED_ENABLE_4_SECTORS
#ifndef GCS_LED_ENABLE_8_SECTORS
	#error "Must define either GCS_LED_ENABLE_4_SECTORS or GCS_LED_ENABLE_8_SECTORS!"
#endif
#endif

// Use GPIO_GCS_TTL1 and GPIO_GCS_TTL2 outputs to control 4 LEDs
#ifdef  GCS_LED_ENABLE_4_SECTORS
	#define GCS_LED_NUM_CASES		4				// Number of different cases
	#define GCS_LED_NUM_OUTPUTS		2				// Number of LED outputs to control per case

	// The compiler is very fusy about multiline preprocessor macros as defined below
    // There must be no whitespace after the \ character which advices macro continues on next line
    // Also comments must end before the \ character hence why use /* Comment */
    #define GCS_LED_MATRIX_ARRAY    {1, 1},      	/* Case 1: North               	*/ \
									{1, 0},      	/* Case 2: East 				*/ \
					                {0, 1},      	/* Case 3: South             	*/ \
					                {0, 0}       	/* Case 4: West 	        	*/

#endif

// Use GPIO_GCS_TTL1, GPIO_GCS_TTL2, and GPIO_GCS_TTL3 outputs to control 8 LEDs
#ifdef  GCS_LED_ENABLE_8_SECTORS
	#define GCS_LED_NUM_CASES		8		// Number of different cases
	#define GCS_LED_NUM_OUTPUTS		3		// Number of LED outputs to control per case

	// The compiler is very fusy about multiline preprocessor macros as defined below
	// There must be no whitespace after the \ character which advices macro continues on next line
	// Also comments must end before the \ character hence why use /* Comment */
	#define GCS_LED_MATRIX_ARRAY    {1, 1, 1},      /* Case 1: North               	*/ \
									{0, 1, 1},      /* Case 2: North > 45 degrees	*/ \
					                {1, 1, 0},      /* Case 3: East             	*/ \
					                {0, 1, 0},      /* Case 4: East > 45        	*/ \
					                {1, 0, 1},      /* Case 5: South            	*/ \
					                {0, 0, 1},      /* Case 6: South > 45       	*/ \
					                {1, 0, 0},      /* Case 7: West             	*/ \
					                {0, 0, 0}       /* Case 8: West > 45 			*/


#endif

//////////////////////////////////////////////////////////////////////////////
// RC Channel / GPIO output definitions
//

// Using APM 2.5 Clone from HobbyKing
#define GPIO_CH0		54		// A0
#define GPIO_CH1    	55		// A1
#define GPIO_CH2    	56		// A2
#define GPIO_CH3    	57		// A3
#define GPIO_CH4    	58
#define GPIO_CH5    	59
#define GPIO_CH6    	60
#define GPIO_CH7    	61
#define GPIO_CH8    	62
#define GPIO_CH9    	13
#define GPIO_CH10   	45		// A10

// Pixhawk1
#define GPIO_AUX1   	50		// AUX1
#define GPIO_AUX2   	51		// AUX2
#define GPIO_AUX3   	52		// AUX3
#define GPIO_AUX4   	53		// AUX4
#define GPIO_AUX5   	54		// AUX5
#define GPIO_AUX6   	55		// AUX6

// PWM LED Definitions
#ifdef GCS_LED_ENABLE_4_SECTORS
	#define PWM_NORTH 		1100
	#define PWM_EAST 		1200
	#define PWM_SOUTH 		1300
	#define PWM_WEST 		1400
// Not used but define anyway as are in code
	//#define PWM_NORTH_UP 	900
	//#define PWM_EAST_UP 	910
	//#define PWM_SOUTH_UP 	920
	//#define PWM_WEST_UP 	930
#endif

#ifdef GCS_LED_ENABLE_8_SECTORS
	#define PWM_NORTH 		1100
	#define PWM_NORTH_UP 	1200
	#define PWM_EAST 		1700
	#define PWM_EAST_UP 	1800
	#define PWM_SOUTH 		1500
	#define PWM_SOUTH_UP 	1600
	#define PWM_WEST 		1300
	#define PWM_WEST_UP 	1400
#endif

#ifdef APM25_CLONE	
	// Using APM 2.5 Clone from HobbyKing
	#define PWM_CH_OUT		3 		// Single PWM output channel number
	
	#define GPIO_GCS_TTL1	GPIO_CH1
	#define GPIO_GCS_TTL2	GPIO_CH2
	#define GPIO_GCS_TTL3	GPIO_CH3

#else	
	// Using Pixhawk
	#define PWM_CH_OUT		CH_9 			// Single PWM output CH_9 = AUX1

#ifdef GCS_LED_ENABLE_4_SECTORS
	#define GPIO_GCS_TTL1	GPIO_AUX5
	#define GPIO_GCS_TTL2	GPIO_AUX6
#endif
#ifdef GCS_LED_ENABLE_8_SECTORS
	#define GPIO_GCS_TTL1	GPIO_AUX4
	#define GPIO_GCS_TTL2	GPIO_AUX5
	#define GPIO_GCS_TTL3	GPIO_AUX6
#endif

#endif

// Direction 
enum SectorDirection {
    SECTOR_DIR_NORTH=1,
    SECTOR_DIR_EAST=2,
    SECTOR_DIR_SOUTH=3,
    SECTOR_DIR_WEST=4
};

enum SectorPitch {
    SECTOR_PITCH_OVER45=0,
    SECTOR_PITCH_UNDER45=1
};

#endif // _GCS_LED_H