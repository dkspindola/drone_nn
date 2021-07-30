# include "../crazyflie-firmware/vendor/FreeRTOS/include/FreeRTOS.h"
# include <stdio.h>
#include <math.h>
#include "../crazyflie-firmware/src/modules/interface/param.h"
#include "../crazyflie-firmware/src/modules/interface/log.h"
#include "../crazyflie-firmware/src/modules/interface/stabilizer.h"
#include "../crazyflie-firmware/src/modules/interface/stabilizer_types.h"
#include "../crazyflie-firmware/src/modules/interface/estimator_kalman.h"
#include "../crazyflie-firmware/src/modules/interface/kalman_core/kalman_core.h"
#include "../crazyflie-firmware/src/modules/interface/commander.h"
#include "NN_ME.h"
#include <stdint.h>
#include <stdbool.h>
#include "myHeader.h"
#include "../crazyflie-firmware/src/drivers/interface/motors.h"
# include "../crazyflie-firmware/vendor/FreeRTOS/include/task.h"
#include "../crazyflie-firmware/src/config/FreeRTOSConfig.h"
#include "../crazyflie-firmware/src/modules/interface/power_distribution.h"
#include "NN_S2M.h"

static int8_t signal;
static setpoint_t point;

static double actions[4];
static point_t kalmanPosition;
static point_t kalmanVelocity;
static point_t kalmanAttitude;
// static float kalmanRotationMatrix[3][3];
static double rotation_matrix[3][3];
static double roll;
static double pitch;
static double yaw;
static float roll_rate;
static float pitch_rate;
static float yaw_rate;

// define your nn here
static const int structure[3][2] = {{64, 18},{64, 64},{8, 64}};
static double state_array[18];
static double hidden_layer0[64];
static double hidden_layer1[64];
static double output_layer[8];

// write the functions for your nn here
static void calc_actions() {

		for (int i = 0; i < structure[0][0]; i++) {
			for (int j = 0; j < structure[0][1]; j++) {
				hidden_layer0[i] += HIDDEN_WEIGHTS0[i][j] * state_array[j];
			}
			hidden_layer0[i] += HIDDEN_BIASES0[i];
			hidden_layer0[i] = tanh(hidden_layer0[i]);
		}
	
		for (int i = 0; i < structure[1][0]; i++) {
			for (int j = 0; j < structure[1][1]; j++) {
				hidden_layer1[i] += HIDDEN_WEIGHTS1[i][j] * hidden_layer0[j];
			}
			hidden_layer1[i] += HIDDEN_BIASES1[i];
			hidden_layer1[i] = tanh(hidden_layer1[i]);
		}
		
		for (int i = 0; i < structure[2][0]; i++) {
			for (int j = 0; j < structure[2][1]; j++) {
				output_layer[i] += OUTPUT_WEIGHTS[i][j] * hidden_layer1[j];
			}
			output_layer[i] += OUTPUT_BIASES[i];
		}
		
		actions[0] = output_layer[0];
		actions[1] = output_layer[1];
		actions[2] = output_layer[2];
		actions[3] = output_layer[3];
	}
/*
static void networkEvaluate_S2M() {

    for (int i = 0; i < structure[0][1]; i++) {
        hidden_layer0[i] = 0;
        for (int j = 0; j < structure[0][0]; j++) {
            hidden_layer0[i] += state_array[j] * HIDDEN_WEIGHTS0_S2M[j][i];
        }
        hidden_layer0[i] += HIDDEN_BIASES0_S2M[i];
        hidden_layer0[i] = tanh(hidden_layer0[i]);
    }

    for (int i = 0; i < structure[1][1]; i++) {
        hidden_layer1[i] = 0;
        for (int j = 0; j < structure[1][0]; j++) {
            hidden_layer1[i] += hidden_layer0[j] * HIDDEN_WEIGHTS1_S2M[j][i];
        }
        hidden_layer1[i] += HIDDEN_BIASES1_S2M[i];
        hidden_layer1[i] = tanh(hidden_layer1[i]);
    }

    for (int i = 0; i < structure[2][1]; i++) {
        output_layer[i] = 0;
        for (int j = 0; j < structure[2][0]; j++) {
            output_layer[i] += hidden_layer1[j] * OUTPUT_WEIGHTS_S2M[j][i];
        }
        output_layer[i] += OUTPUT_BIASES_S2M[i];
    }

    actions[0] = output_layer[0];
    actions[1] = output_layer[1];
    actions[2] = output_layer[2];
    actions[3] = output_layer[3];

}
 */

void calc_rot_matrix() {
    // rotation around x-axis
    double r_x[3][3] = {{1,         0,          0},
                        {0, cos(roll), -sin(roll)},
                        {0, sin(roll),  cos(roll)}};

    // rotation around y-axis
    double r_y[3][3] = {{cos(pitch),  0, sin(pitch)},
                        {0,           1,          0},
                        {-sin(pitch), 0, cos(pitch)}};

    // rotation around z-axis
    double r_z[3][3] = {{cos(yaw), -sin(yaw), 0},
                        {sin(yaw),  cos(yaw), 0},
                        {0,                0, 1}};

    double r_tmp[3][3];
    for (int i = 0; i < 3; i++) {
        for (int j = 0; j < 3; j++) {
            double sum1 = 0;
            for (int k = 0; k < 3; k++) {
                sum1 += r_z[i][k] * r_y[k][j];
            }
            r_tmp[i][j] = sum1;
        }
    }

    for (int i = 0; i < 3; i++) {
        for (int j = 0; j < 3; j++) {
            double sum2 = 0;
            for (int k = 0; k < 3; k++) {
                sum2 += r_tmp[i][k] * r_x[k][j];
            }
            rotation_matrix[i][j] = sum2;
        }
    }
}

static void get_state() {
    estimatorKalmanGetEstimatedPos(&kalmanPosition); // estimate position in world frame

    estimatorKalmanGetEstimatedVel(&kalmanVelocity); // estimate velocity in body frame

    // estimatorKalmanGetEstimatedRot(*kalmanRotationMatrix);
    estimatorKalmanGetEstimatedAtt(&kalmanAttitude);
    roll = kalmanAttitude.x;
    pitch = kalmanAttitude.y;
    yaw = kalmanAttitude.z;
    calc_rot_matrix();

    stabilizerGetRollRate(&roll_rate);
    stabilizerGetPitchRate(&pitch_rate);
    stabilizerGetYawRate(&yaw_rate);

    state_array[0] = kalmanPosition.x;
    state_array[1] = kalmanPosition.y;
    state_array[2] = kalmanPosition.z - 2;
    state_array[3] = kalmanVelocity.x;
    state_array[4] = kalmanVelocity.y;
    state_array[5] = kalmanVelocity.z;
    state_array[6] = rotation_matrix[0][0];
    state_array[7] = rotation_matrix[0][1];
    state_array[8] = rotation_matrix[0][2];
    state_array[9] = rotation_matrix[1][0];
    state_array[10] = rotation_matrix[1][1];
    state_array[11] = rotation_matrix[1][2];
    state_array[12] = rotation_matrix[2][0];
    state_array[13] = rotation_matrix[2][1];
    state_array[14] = rotation_matrix[2][2];
    state_array[15] = roll_rate / 1000;
    state_array[16] = pitch_rate / 1000;
    state_array[17] = yaw_rate / 1000;
}

// other functions

static void set_thrust() {
    uint16_t thrust[4];
    for (int i = 0; i < 4; i++) {
        thrust[i] = (uint16_t) ((tanh(actions[i]) + 1) * 32767);
    }
    setMotorPowerSet(thrust);
}

void start_motors() {
    uint16_t thrust[4] = {65535, 65535, 65535, 65535};
    setMotorPowerSet(thrust);
}

void shutoff_motors()
{
    uint16_t thrust[4] = {0, 0, 0, 0};
    setMotorPowerSet(thrust);
}

void fly()
{
    get_state();
    calc_actions();
    set_thrust();
}

void test() {
    actions[0] = 10;
    actions[1] = 10;
    actions[2] = 10;
    actions[3] = 10;
    set_thrust();
}

static void setHoverSetpointPos(setpoint_t *setpoint, float x, float y, float z)
{
    setpoint->mode.z = modeAbs;
    setpoint->mode.x = modeAbs;
    setpoint->mode.y = modeAbs;

    setpoint->position.z = z;
    setpoint->position.x = x;
    setpoint->position.y = y;

}

void changeState() {
    // read droneCmd set from the ground station to handle commands
    switch (signal) {
        case 1:
            fly();
            break;
        case 2:
            setHoverSetpointPos(&point, kalmanPosition.x, kalmanPosition.y, -1);
            commanderSetSetpoint(&point, 3);
            break;
        default:
            shutoff_motors();
            break;
    } //end of switch(droneCmd)
}

PARAM_GROUP_START(control)
PARAM_ADD_GROUP(LOG_INT8, signal, &signal)
PARAM_GROUP_STOP(control)

LOG_GROUP_START(state)
LOG_ADD(LOG_FLOAT, pos_err_x, &state_array[0])
LOG_ADD(LOG_FLOAT, pos_err_y, &state_array[1])
LOG_ADD(LOG_FLOAT, pos_err_z, &state_array[2])
LOG_ADD(LOG_FLOAT, lin_vel_err_x, &state_array[3])
LOG_ADD(LOG_FLOAT, lin_vel_err_y, &state_array[4])
LOG_ADD(LOG_FLOAT, lin_vel_err_z, &state_array[5])
LOG_ADD(LOG_FLOAT, rot_matrix_00, &state_array[6])
LOG_ADD(LOG_FLOAT, rot_matrix_01, &state_array[7])
LOG_ADD(LOG_FLOAT, rot_matrix_02, &state_array[8])
LOG_ADD(LOG_FLOAT, rot_matrix_10, &state_array[9])
LOG_ADD(LOG_FLOAT, rot_matrix_11, &state_array[10])
LOG_ADD(LOG_FLOAT, rot_matrix_12, &state_array[11])
LOG_ADD(LOG_FLOAT, rot_matrix_20, &state_array[12])
LOG_ADD(LOG_FLOAT, rot_matrix_21, &state_array[13])
LOG_ADD(LOG_FLOAT, rot_matrix_22, &state_array[14])
LOG_ADD(LOG_FLOAT, ang_vel_err_x, &state_array[15])
LOG_ADD(LOG_FLOAT, ang_vel_err_y, &state_array[16])
LOG_ADD(LOG_FLOAT, ang_vel_err_z, &state_array[17])
LOG_GROUP_STOP(state)

LOG_GROUP_START(actions)
LOG_ADD(LOG_FLOAT, actions_0, &actions[0])
LOG_ADD(LOG_FLOAT, actions_1, &actions[1])
LOG_ADD(LOG_FLOAT, actions_2, &actions[2])
LOG_ADD(LOG_FLOAT, actions_3, &actions[3])
LOG_GROUP_STOP(actions)
