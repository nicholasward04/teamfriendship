/*
 * Module for the implementation of motor driving
 */
#include "main.h"
#include "Motors.h"
#include "math.h"
#include "IR.h"

extern TIM_HandleTypeDef htim2;
extern TIM_HandleTypeDef htim3;
extern TIM_HandleTypeDef htim4;

bool moving_forw = false;
bool moving_back = false;

extern float nom_FL;
extern float nom_FR;
float FRONT_THRESH_L = 4800;
float FRONT_THRESH_R = 2600;
uint16_t counter_L  = 0;
uint16_t counter_R  = 0;
int32_t objective_L = 0;
int32_t objective_R = 0;

const float wheel_radius = 16.2; // In mm
const float wheel_circumference = wheel_radius * 3.1415926 * 2; // In mm
const unsigned ticks_per_revolution = 360; // Number of encoder ticks per revolution
const float mm_per_tick = 0.2947; // mm travelled per tick

const float wheel_to_center = 38.8; // Distance between wheel and center of mouse in mm
float global_angle = 0; // Global angle in degrees
float change_in_angle = 0; // Change in global angle in degrees
float start_angle = 0;

float forward_progress = 0;
float mm_to_travel = 0;

float curr_duty_cycle_L = 0;
float curr_duty_cycle_R = 0;

float base_duty_cycle_L = 0;
float base_duty_cycle_R = 0;

extern float prev_error_IR;
extern float prev_error_angle;
float total_error_angle;

extern bool wall_front;

float angle = 0;

void angle_calc(float d_L, float d_R) { // Calculates angle with
	change_in_angle = ((d_R - d_L) / wheel_to_center) * (180 / 3.1415926);
	global_angle = global_angle + change_in_angle;
}

void HAL_TIM_IC_CaptureCallback(TIM_HandleTypeDef *htim) {
	uint16_t current_count = __HAL_TIM_GET_COUNTER(htim);
	int32_t prev_left = objective_L;
	int32_t prev_right = objective_R;
	if (htim == &htim3) { // Motor right timer
		objective_R += (int16_t)(current_count - counter_R);
		counter_R = current_count;
	}
	if (htim == &htim4) { // Motor left timer
		objective_L += (int16_t)(current_count - counter_L);
		counter_L = current_count;
	}

	float distance_L = (objective_L - prev_left) * mm_per_tick / 2;
	float distance_R = (objective_R - prev_right) * mm_per_tick / 2;
	angle_calc(distance_L, distance_R);
}

void set_motor_dir(motor motor_type, mouse_dir dir) {
	switch (motor_type) {
	case MOTOR_RIGHT:
		if (dir == FORWARD) {
			HAL_GPIO_WritePin(MR_FWD_GPIO_Port, MR_FWD_Pin, 1);
			HAL_GPIO_WritePin(MR_BWD_GPIO_Port, MR_BWD_Pin, 0);
		}
		else if (dir == BACKWARD) {
			HAL_GPIO_WritePin(MR_FWD_GPIO_Port, MR_FWD_Pin, 0);
			HAL_GPIO_WritePin(MR_BWD_GPIO_Port, MR_BWD_Pin, 1);
		}
		break;
	case MOTOR_LEFT:
		if (dir == FORWARD) {
			HAL_GPIO_WritePin(ML_FWD_GPIO_Port, ML_FWD_Pin, 1);
			HAL_GPIO_WritePin(ML_BWD_GPIO_Port, ML_BWD_Pin, 0);
		}
		else if (dir == BACKWARD) {
			HAL_GPIO_WritePin(ML_FWD_GPIO_Port, ML_FWD_Pin, 0);
			HAL_GPIO_WritePin(ML_BWD_GPIO_Port, ML_BWD_Pin, 1);
		}
		break;
	default:
		break;
	}
}

void set_dir(mouse_dir dir) { // Set direction for movement
	switch (dir) {
	case FORWARD:
		set_motor_dir(MOTOR_RIGHT, FORWARD);
		set_motor_dir(MOTOR_LEFT, FORWARD);
		break;
	case BACKWARD:
		set_motor_dir(MOTOR_RIGHT, BACKWARD);
		set_motor_dir(MOTOR_LEFT, BACKWARD);
		break;
	case RIGHT:
		set_motor_dir(MOTOR_RIGHT, BACKWARD);
		set_motor_dir(MOTOR_LEFT, FORWARD);
		break;
	case LEFT:
		set_motor_dir(MOTOR_RIGHT, FORWARD);
		set_motor_dir(MOTOR_LEFT, BACKWARD);
		break;
	default:
		break;
	}
}

void set_speed(float duty_cycle, motor motor) { // Set the speed of a motor
	uint16_t pulse_value = duty_cycle * 2047;
	if (duty_cycle > 0.5) {
		pulse_value = 0.5*2047;
	}
	if (motor) { // MOTOR LEFT
		TIM2->CCR4 = pulse_value;
		curr_duty_cycle_L = duty_cycle;
	}
	else { // MOTOR_RIGHT
		TIM2->CCR3 = pulse_value;
		curr_duty_cycle_R = duty_cycle;
	}
}

void turnRight(float speed) {
	set_dir(RIGHT);
	set_speed(speed, MOTOR_LEFT);
	set_speed(speed, MOTOR_RIGHT);
	float start_angle = global_angle;
	while (global_angle - start_angle > -90 + 2);
	set_speed(0, MOTOR_LEFT);
	set_speed(0, MOTOR_RIGHT);
}

void turnLeft(float speed) {
	set_dir(LEFT);
	set_speed(speed, MOTOR_LEFT);
	set_speed(speed, MOTOR_RIGHT);
	float start_angle = global_angle;
	while (global_angle - start_angle < 90 - 2);
	set_speed(0, MOTOR_LEFT);
	set_speed(0, MOTOR_RIGHT);
}

void moveForward(float speed, float mm_to_move) { // 180 mm being standard cell distance
	int32_t start_count = (objective_L + objective_R) / 2;
	int32_t current_count = (objective_L + objective_R) / 2;
	int32_t ticks_to_travel = (mm_to_move / mm_per_tick);

	base_duty_cycle_L = speed;
	base_duty_cycle_R = speed;

	set_dir(FORWARD);
	set_speed(speed, MOTOR_LEFT);
	set_speed(speed, MOTOR_RIGHT);

	mm_to_travel = mm_to_move;

	moving_forw = true;
	while ((current_count-start_count) < ticks_to_travel) {
		if (nom_FR > 200 && nom_FL > 200 && (mm_to_move > 115 || mm_to_move == 75)) {
			break;
		}
		else if (nom_FR > FRONT_THRESH_R && nom_FL > FRONT_THRESH_L && mm_to_move < 115) {
			break;
		}

		forward_progress = current_count - start_count;
		current_count = (objective_L + objective_R) / 2;
	}
	moving_forw = false;

	prev_error_IR = 0;
	prev_error_angle = 0;

	set_speed(0, MOTOR_LEFT);
	set_speed(0, MOTOR_RIGHT);
}

void moveBackward(float speed, float mm_to_move) {
	base_duty_cycle_L = speed;
	base_duty_cycle_R = speed;

	set_dir(BACKWARD);
	set_speed(speed, MOTOR_LEFT);
	set_speed(speed, MOTOR_RIGHT);

	volatile int x = 0;
	while (x < 6000000) {x++;};

	prev_error_IR = 0;
	prev_error_angle = 0;

	set_speed(0, MOTOR_LEFT);
	set_speed(0, MOTOR_RIGHT);
}
