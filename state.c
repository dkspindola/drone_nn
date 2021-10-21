#include "param.h"

#include "power_distribution.h"

#include "state.h"
#include "drone.h"

#include "debug.h"

#include "usec_time.h"

static int signal;

void change() {
    switch (signal)
    {
    case FLY:
        setMotorSetEnable(true);
        fly();
        break;
    
    case LAND:
        setMotorSetEnable(false);
        land();
        break;
    
    case START:
        setMotorSetEnable(false);
        test();
        break;

    case FLY_S2M:
        setMotorSetEnable(true);
        fly_s2m();
        break;

    default:
        setMotorSetEnable(true);
        shutoff_motors();
        get_state();
        break;
    }
}

PARAM_GROUP_START(state)
PARAM_ADD(PARAM_INT8, signal, &signal)
PARAM_GROUP_STOP(state)