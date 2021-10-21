#include <stdint.h>
#include <math.h>

#include "param.h"
#include "log.h"

#include "stabilizer.h"

#include "estimator_kalman.h"

#include "commander.h"

#include "motors.h"
#include "power_distribution.h"

#include "brain.h"
#include "NN_S2M.h"
#include "drone.h"

#include "debug.h"

static float state[18];
static float action[4];

static float hidden_layer_0[64];
static float hidden_layer_1[64];
static float output_layer[8];

static point_t pos;
static velocity_t vel;
static float rotation_matrix[3][3];
static velocity_t ang_vel;

static point_t goal_pos;

static uint16_t thrust[4];

static setpoint_t test_setpoint;

static void networkEvaluate() {

    for (int i = 0; i < structure_s2m[0][1]; i++) {
        hidden_layer_0[i] = 0;
        for (int j = 0; j < structure_s2m[0][0]; j++) {
            hidden_layer_0[i] += state[j] * HIDDEN_WEIGHTS0_S2M[j][i];
        }
        hidden_layer_0[i] += HIDDEN_BIASES0_S2M[i];
        hidden_layer_0[i] = tanhf(hidden_layer_0[i]);
    }

    for (int i = 0; i < structure_s2m[1][1]; i++) {
        hidden_layer_1[i] = 0;
        for (int j = 0; j < structure_s2m[1][0]; j++) {
            hidden_layer_1[i] += hidden_layer_0[j] * HIDDEN_WEIGHTS1_S2M[j][i];
        }
        hidden_layer_1[i] += HIDDEN_BIASES1_S2M[i];
        hidden_layer_1[i] = tanhf(hidden_layer_1[i]);
    }

    for (int i = 0; i < structure_s2m[2][1]; i++) {
        output_layer[i] = 0;
        for (int j = 0; j < structure_s2m[2][0]; j++) {
            output_layer[i] += hidden_layer_1[j] * OUTPUT_WEIGHTS_S2M[j][i];
        }
        output_layer[i] += OUTPUT_BIASES_S2M[i];
    }

    action[0] = output_layer[0];
    action[1] = output_layer[1];
    action[2] = output_layer[2];
    action[3] = output_layer[3];

}


// write the functions for your nn here
void calc_action() {

    for (int i = 0; i < structure[0][0]; i++) {
        hidden_layer_0[i] = 0;
        for (int j = 0; j < structure[0][1]; j++) {
            hidden_layer_0[i] += HIDDEN_WEIGHTS_0[i][j] * state[j];
        }
        hidden_layer_0[i] += HIDDEN_BIASES_0[i];
        hidden_layer_0[i] = tanhf(hidden_layer_0[i]);
    }
	
    for (int i = 0; i < structure[1][0]; i++) {
        hidden_layer_1[i] = 0;
        for (int j = 0; j < structure[1][1]; j++) {
            hidden_layer_1[i] += HIDDEN_WEIGHTS_1[i][j] * hidden_layer_0[j];
        }
        hidden_layer_1[i] += HIDDEN_BIASES_1[i];
        hidden_layer_1[i] = tanhf(hidden_layer_1[i]);
    }

    for (int i = 0; i < structure[2][0]; i++) {
        output_layer[i] = 0;
        for (int j = 0; j < structure[2][1]; j++) {
            output_layer[i] += OUTPUT_WEIGHTS[i][j] * hidden_layer_1[j];
        }
        output_layer[i] += OUTPUT_BIASES[i];
    }

    action[0] = output_layer[0];
    action[1] = output_layer[1];
    action[2] = output_layer[2];
    action[3] = output_layer[3];
}

void get_state() {

    // get position in global frame [m]
    estimatorKalmanGetEstimatedPos(&pos);
    // get velocity in global frame [m/s]
    stabilizerGetVel(&vel);
    // get rotation matrix from body to global frame []
    estimatorKalmanGetEstimatedRot(*rotation_matrix);
    // get angular velocity in body frame [deg/s]
    stabilizerGetAngVel(&ang_vel);

    state[0] = pos.x - goal_pos.x;
    state[1] = pos.y - goal_pos.y;
    state[2] = pos.z - goal_pos.z;
    state[3] = vel.x - 0;
    state[4] = vel.y - 0;
    state[5] = vel.z - 0;
    state[6] = rotation_matrix[0][0];
    state[7] = rotation_matrix[0][1];
    state[8] = rotation_matrix[0][2];
    state[9] = rotation_matrix[1][0];
    state[10] = rotation_matrix[1][1];
    state[11] = rotation_matrix[1][2];
    state[12] = rotation_matrix[2][0];
    state[13] = rotation_matrix[2][1];
    state[14] = rotation_matrix[2][2];
    state[15] = ang_vel.x / 180 * (float) M_PI - 0;
    state[16] = ang_vel.y / 180 * (float) M_PI - 0;
    state[17] = ang_vel.z / 180 * (float) M_PI - 0;
}

// other functions

static void set_thrust() {
    
    for (int i = 0; i < 4; i++) {
        thrust[i] = (uint16_t) ((tanhf(action[i]) + 1) * 32767);
    }
    setMotorPowerSet(thrust);
}

void shutoff_motors()
{
    for(int i = 0; i < 4; i++) {
        thrust[i] = 0;
    }
    
    setMotorPowerSet(thrust);
}

void fly()
{
    get_state();
    calc_action();
    set_thrust();
}

void fly_s2m() 
{
    get_state();
    networkEvaluate();
    set_thrust();
}

static void setHoverSetpoint(setpoint_t *setpoint, float x, float y, float z)
{
  setpoint->mode.z = modeAbs;
  setpoint->position.x = x;

  setpoint->mode.z = modeAbs;
  setpoint->position.y = y;

  setpoint->mode.z = modeAbs;
  setpoint->position.z = z;

  // setpoint->mode.yaw = modeAbs;
  // setpoint->attitude.yaw = 0;
}

void test() {
    //estimatorKalmanGetEstimatedPos(&pos);
    //calc_action();
    setHoverSetpoint(&test_setpoint, goal_pos.x, goal_pos.y, goal_pos.z);
    commanderSetSetpoint(&test_setpoint, 3);
    //networkEvaluate();
}

void land() {
    estimatorKalmanGetEstimatedPos(&pos);
    if (pos.z < 0.1f) {
        setMotorSetEnable(true);
        shutoff_motors();
    }
    else {
        setHoverSetpoint(&test_setpoint, pos.x, pos.y, -1);
        commanderSetSetpoint(&test_setpoint, 3);
    }
}

LOG_GROUP_START(my_state)
LOG_ADD(LOG_FLOAT, pos_err_x, &state[0])
LOG_ADD(LOG_FLOAT, pos_err_y, &state[1])
LOG_ADD(LOG_FLOAT, pos_err_z, &state[2])
LOG_ADD(LOG_FLOAT, lin_vel_err_x, &state[3])
LOG_ADD(LOG_FLOAT, lin_vel_err_y, &state[4])
LOG_ADD(LOG_FLOAT, lin_vel_err_z, &state[5])
LOG_ADD(LOG_FLOAT, rot_matrix_00, &state[6])
LOG_ADD(LOG_FLOAT, rot_matrix_01, &state[7])
LOG_ADD(LOG_FLOAT, rot_matrix_02, &state[8])
LOG_ADD(LOG_FLOAT, rot_matrix_10, &state[9])
LOG_ADD(LOG_FLOAT, rot_matrix_11, &state[10])
LOG_ADD(LOG_FLOAT, rot_matrix_12, &state[11])
LOG_ADD(LOG_FLOAT, rot_matrix_20, &state[12])
LOG_ADD(LOG_FLOAT, rot_matrix_21, &state[13])
LOG_ADD(LOG_FLOAT, rot_matrix_22, &state[14])
LOG_ADD(LOG_FLOAT, ang_vel_err_x, &state[15])
LOG_ADD(LOG_FLOAT, ang_vel_err_y, &state[16])
LOG_ADD(LOG_FLOAT, ang_vel_err_z, &state[17])
LOG_GROUP_STOP(my_state)

LOG_GROUP_START(my_action)
LOG_ADD(LOG_FLOAT, action_0, &action[0])
LOG_ADD(LOG_FLOAT, action_1, &action[1])
LOG_ADD(LOG_FLOAT, action_2, &action[2])
LOG_ADD(LOG_FLOAT, action_3, &action[3])
LOG_GROUP_STOP(my_action)

LOG_GROUP_START(my_thrust)
LOG_ADD(LOG_UINT16, thrust_0, &thrust[0])
LOG_ADD(LOG_UINT16, thrust_1, &thrust[1])
LOG_ADD(LOG_UINT16, thrust_2, &thrust[2])
LOG_ADD(LOG_UINT16, thrust_3, &thrust[3])
LOG_GROUP_STOP(my_thrust)

PARAM_GROUP_START(state_pos)
PARAM_ADD(PARAM_FLOAT, x, &(goal_pos.x))
PARAM_ADD(PARAM_FLOAT, y, &(goal_pos.y))
PARAM_ADD(PARAM_FLOAT, z, &(goal_pos.z))
PARAM_GROUP_STOP(state_pos)
