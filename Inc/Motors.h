/*
 * Module for the implementation of motor driving and control
 */

#ifndef __Motors_H
#define __Motors_H

typedef enum {
	FORWARD,
	BACKWARD,
	RIGHT,
	LEFT,
	NONE
} mouse_dir;

typedef enum {
	MOTOR_RIGHT,
	MOTOR_LEFT
} motor;

void HAL_TIM_IC_CaptureCallback(TIM_HandleTypeDef *htim);

void set_motor_dir(motor motor_type, mouse_dir dir);
void set_dir(mouse_dir dir);
void set_speed(float duty_cycle, motor motor);

void angle_calc(float d_L, float d_R);

void turnRight(float speed);
void turnLeft(float speed);
void moveForward(float speed, float mm_to_move);
void moveBackward(float speed, float mm_to_move);

#endif
