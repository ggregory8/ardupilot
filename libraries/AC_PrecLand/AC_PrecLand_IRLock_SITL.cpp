/// -*- tab-width: 4; Mode: C++; c-basic-offset: 4; indent-tabs-mode: nil -*-
#include <AP_HAL/AP_HAL.h>
#include <AC_PrecLand/AC_PrecLand_IRLock_SITL.h>

extern const AP_HAL::HAL& hal;

// Constructor
AC_PrecLand_IRLock_SITL::AC_PrecLand_IRLock_SITL(const AC_PrecLand& frontend, AC_PrecLand::precland_state& state)
    : AC_PrecLand_Backend(frontend, state)//,
      //irlock()
{
}

// init - perform initialisation of this backend
void AC_PrecLand_IRLock_SITL::init()
{
    //irlock.init();

    // set healthy
    //_state.healthy = irlock.healthy();
    _state.healthy = true;
}

// update - give chance to driver to get updates from sensor
bool AC_PrecLand_IRLock_SITL::update()
{
    // get new sensor data
    //return (irlock.update());
    check_lock();

    return true;
}

// get_angle_to_target - returns body frame angles (in radians) to target
//  returns true if angles are available, false if not (i.e. no target)
//  x_angle_rad : body-frame roll direction, positive = target is to right (looking down)
//  y_angle_rad : body-frame pitch direction, postiive = target is forward (looking down)
bool AC_PrecLand_IRLock_SITL::get_angle_to_target(float &x_angle_rad, float &y_angle_rad) const
{
    //return irlock.get_angle_to_target(x_angle_rad, y_angle_rad);
    return true;
}

// Check for timeout (i.e no getting update for a certain time)
void AC_PrecLand_IRLock_SITL::check_lock()
{
    if (_state.sim_lock) {
        
        _state.locked = true;
        
        _irlock_i = 0;
    } else {
        _irlock_i = _irlock_i + 1;
        // If no new IR-Lock data received after timeout value (PRECLAND_TIMEOUT parameter) then set locked state false
        if (_irlock_i >= (_state.timeout_ms / 20))
        {
            _state.locked = false;
        }
    }
}