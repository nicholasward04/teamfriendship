/*
 * Module for the implementation of motor control
 */
 
#include "Control.h"
#include "IR.h"
#include "Motors.h"
#include "main.h"

extern bool initial_wall_L;
extern bool initial_wall_R;

extern float curr_duty_cycle_L;
extern float curr_duty_cycle_R;

extern float base_duty_cycle_L;
extern float base_duty_cycle_R;

extern float objective_L;
extern float objective_R;
extern float mm_per_tick;

extern float nom_L;
extern float nom_R;
extern float nom_FL;
extern float nom_FR;

float error_left_IR;
float error_right_IR;
float total_error_IR;
float prev_error_IR = 0;

extern float total_error_angle;
float prev_error_angle = 0;

extern bool wall_left;
extern bool wall_right;
extern bool wall_front;

extern float global_angle;
extern float change_in_angle;
extern float start_angle;
extern float mouse_angle;

extern bool moving_forw;
extern bool moving_back;

const uint16_t counter_period = 2047;
float current_voltage = 8.4; // In volts

const float kp_IR = 0.005; // 0.005
const float kd_IR = 0.0005;

const float kp_angle = 0.1;
const float kd_angle = 0;

extern float forward_progress;
extern float mm_to_travel;

extern float FRONT_THRESH_L;
extern float FRONT_THRESH_R;

int32_t previous_obj_L = 0;
int32_t previous_obj_R = 0;

void angular_control() {
//	total_error_angle = mouse_angle - global_angle;
//
//	float voltage_adjustment = (total_error_angle * kp_angle);
//
//	if (voltage_adjustment > 0.6) {
//		voltage_adjustment = 0.6;
//	}

//	curr_duty_cycle_L += (voltage_adjustment / current_voltage);
//	curr_duty_cycle_R -= (voltage_adjustment / current_voltage);

	curr_duty_cycle_L -= -.02;
	curr_duty_cycle_R += 0.02;

	set_speed(curr_duty_cycle_L, MOTOR_LEFT);
	set_speed(curr_duty_cycle_R, MOTOR_RIGHT);
}
 
void updateVoltage() {
	wall_left = wallLeft();
	wall_right = wallRight();

//	float distance_L = (objective_L - previous_obj_L) * mm_per_tick / 2;
//	float distance_R = (objective_R - previous_obj_R) * mm_per_tick / 2;
//	angle_calc(distance_L, distance_R);

	if (nom_L < 40) {
		wall_left = false;
	}
	if (nom_R < 40) {
		wall_right = false;
	}

	curr_duty_cycle_L = base_duty_cycle_L;
	curr_duty_cycle_R = base_duty_cycle_R;

	if (!initial_wall_L && !initial_wall_R && !moving_back) { // Angular control system
		angular_control();
	}
	else { // IR control system
		error_left_IR = 100.f - nom_L;
		error_right_IR = 100.f - nom_R;
		total_error_IR = 0;

//		if (!wall_front) {
		if (wall_left && wall_right) {
			if (nom_L < 100 && nom_R < 100) {
				angular_control();
				total_error_IR = 0;
			}
			else {
				total_error_IR = error_right_IR - error_left_IR;
			}
		}
		else if (wall_left && initial_wall_L) { // && initial_wall_L
			total_error_IR = -2 * error_left_IR;
		}
		else if (wall_right && initial_wall_R) { // && initial_wall_R
			total_error_IR = 2 * error_right_IR;
		}
//		}
//		else {
//			if (mm_to_travel == 114.5754) {
//				error_right_IR = FRONT_THRESH_R - nom_FR;
//				error_left_IR = FRONT_THRESH_L - nom_FL;
//			}
//			else {
//				error_right_IR = 200.f - nom_FR;
//				error_left_IR = 200.f - nom_FL;
//			}
//			total_error_IR = (error_left_IR - error_right_IR) / 5;
//		}

		float voltage_adjustment = (total_error_IR * kp_IR);

		if ((forward_progress > (mm_to_travel - 80) / mm_per_tick)
				&& ((!initial_wall_L && !wall_left) || (!initial_wall_R && !wall_right)) ) {
			angular_control();
			voltage_adjustment = 0;
		}

		if (voltage_adjustment > 0.6) {
			voltage_adjustment = 0.6;
		}
		else if (voltage_adjustment < -0.6) {
			voltage_adjustment = -0.6;
		}

		if (moving_forw) {
			curr_duty_cycle_L -= (voltage_adjustment / current_voltage);
			curr_duty_cycle_R += (voltage_adjustment / current_voltage);
		}

		set_speed(curr_duty_cycle_L, MOTOR_LEFT);
		set_speed(curr_duty_cycle_R, MOTOR_RIGHT);
	}
	previous_obj_L = objective_L;
	previous_obj_R = objective_R;
}
