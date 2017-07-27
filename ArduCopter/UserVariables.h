// user defined variables

// example variables used in Wii camera testing - replace with your own
// variables
#ifdef USERHOOK_VARIABLES

#if WII_CAMERA == 1
WiiCamera           ircam;
int                 WiiRange=0;
int                 WiiRotation=0;
int                 WiiDisplacementX=0;
int                 WiiDisplacementY=0;
#endif  // WII_CAMERA

////////////////////////////////////////////////////////////////////////////////
// GG - Allow return to me (moving home location)
////////////////////////////////////////////////////////////////////////////////
//bool     rtm_update_location;           // Home location needs to be changed even if RTL already active (// Used for IR-Lock only?)
float    rtm_distance;                  // Distance to Return-to-me home location
float    rtm_bearing;                   // Bearing to Return-to-me home location

//uint16_t rtm_update_location_counter;   // Allow a couple of loops before running the update_RTL_location() function otherwise the current_loc.alt hasn't updated correctly 

//#define RTM_ABORT_LAND_ALT 500              // Altitude (cm) where we can abort RTL land (and switch to GUIDED) if we lose IR-Lock fix. If below this altitude it will continue landing
//#define RTM_IGNORE_BEACAN_ALT 200           // Altitude (cm) where we can ignore issuing warning if we lose IR-Lock fix as beacon will be out of view of sensor
//#define RTM_LOITER_TIME 5000                // Time (ms) to loiter at RTM_LOITER_ALT before checking IR-Lock fix

#endif  // USERHOOK_VARIABLES


