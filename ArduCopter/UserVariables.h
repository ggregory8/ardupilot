/// -*- tab-width: 4; Mode: C++; c-basic-offset: 4; indent-tabs-mode: nil -*-

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
struct   Location rtm_loc;             	// Return to me location
bool     rtm_allow;            			// Return to me location recieved, allow moving RTL
bool     rtm_update_location;  			// Home location needs to be changed even if RTL already active
float    rtm_distance;                 	// Distance to Return-to-me home location
float    rtm_bearing;                  	// Bearing to Return-to-me home location
bool     irlock_fix_last;      			// track last state of irlock fix
bool     irlock_rtl_pause;     			// Keep track if we lose IR-lock during RTL land, this allows us to resume if we regain IR-Lock fix
bool     irlock_land_pause;    			// Keep track if we lose IR-lock during RTL land, this allows us to resume if we regain IR-Lock fix

bool 	irlock_blob_detected;

bool 	land_repo_active_last;			// Tack last state of ap.land_repo_active

#endif  // USERHOOK_VARIABLES


