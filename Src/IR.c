/*
 * Module for the implementation my mouse' IR sensors and their normalization
 */
#include "IR.h"

extern ADC_HandleTypeDef hadc1;

bool wall_left = false;
bool wall_right = false;
bool wall_front = false;

// Normalize IR values
// Declarations of raw IR values (calibration values)
uint16_t raw_FL = 0;
uint16_t raw_FR = 0;
uint16_t raw_L = 0;
uint16_t raw_R = 0;
// Declare calibration values (to be determined when testing)
const float cal_FL = 61;
const float cal_FR = 105;
const float cal_L = 367;
const float cal_R = 820;
// Declare nominal distance values
float nom_FL;
float nom_FR;
float nom_L;
float nom_R;
// Declare scale values
float scale_FL = 200/cal_FL;
float scale_FR = 200/cal_FR;
float scale_L  = 100/cal_L;
float scale_R  = 100/cal_R;

int on = 0;

void ADC1_Select_CH5(void) {
	ADC_ChannelConfTypeDef sConfig = {0};

	sConfig.Channel = ADC_CHANNEL_5;
	sConfig.Rank = ADC_REGULAR_RANK_1;
	sConfig.SamplingTime = ADC_SAMPLETIME_7CYCLES_5;
	if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK) {
		Error_Handler();
	}
}
void ADC1_Select_CH8(void) {
	ADC_ChannelConfTypeDef sConfig = {0};

	sConfig.Channel = ADC_CHANNEL_8;
	sConfig.Rank = ADC_REGULAR_RANK_1;
	sConfig.SamplingTime = ADC_SAMPLETIME_7CYCLES_5;
	if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK) {
		Error_Handler();
	}
}
void ADC1_Select_CH4(void) {
	ADC_ChannelConfTypeDef sConfig = {0};

	sConfig.Channel = ADC_CHANNEL_4;
	sConfig.Rank = ADC_REGULAR_RANK_1;
	sConfig.SamplingTime = ADC_SAMPLETIME_7CYCLES_5;
	if (HAL_ADC_ConfigChannel (&hadc1, &sConfig) != HAL_OK) {
		Error_Handler();
	}
}
void ADC1_Select_CH9(void) { // Gather raw IR values
	ADC_ChannelConfTypeDef sConfig = {0};

	sConfig.Channel = ADC_CHANNEL_9;
	sConfig.Rank = ADC_REGULAR_RANK_1;
	sConfig.SamplingTime = ADC_SAMPLETIME_7CYCLES_5;
	if (HAL_ADC_ConfigChannel (&hadc1, &sConfig) != HAL_OK) {
		Error_Handler();
	}
}

unsigned measure_dist(dist_t dist) { // Poll raw IR sensors
	GPIO_TypeDef* emitter_port;
	uint16_t emitter_pin;
	GPIO_TypeDef* receiver_port;
	uint16_t receiver_pin;

	switch(dist) {
	case DIST_FL:
		emitter_port = EMIT_FL_GPIO_Port;
		emitter_pin = EMIT_FL_Pin;
		receiver_port = RECIV_FL_GPIO_Port;
		receiver_pin = RECIV_FL_Pin;
		ADC1_Select_CH8();
		break;
	case DIST_FR:
		emitter_port = EMIT_FR_GPIO_Port;
		emitter_pin = EMIT_FR_Pin;
		receiver_port = RECIV_FR_GPIO_Port;
		receiver_pin = RECIV_FR_Pin;
		ADC1_Select_CH5();
		break;
	case DIST_R:
		emitter_port = EMIT_R_GPIO_Port;
		emitter_pin = EMIT_R_Pin;
		receiver_port = RECIV_R_GPIO_Port;
		receiver_pin = RECIV_R_Pin;
		ADC1_Select_CH4();
		break;
	case DIST_L:
		emitter_port = EMIT_L_GPIO_Port;
		emitter_pin = EMIT_L_Pin;
		receiver_port = RECIV_L_GPIO_Port;
		receiver_pin = RECIV_L_Pin;
		ADC1_Select_CH9();
		break;
	default:
		break;
	}
	HAL_GPIO_WritePin(emitter_port, emitter_pin, GPIO_PIN_SET);
	//HAL_Delay(2);

	HAL_ADC_Start(&hadc1);
	HAL_ADC_PollForConversion(&hadc1, HAL_MAX_DELAY);
	uint16_t adc_val = HAL_ADC_GetValue(&hadc1);
	HAL_ADC_Stop(&hadc1);

	HAL_GPIO_WritePin(emitter_port, emitter_pin, GPIO_PIN_RESET);

	return adc_val;
}

void poll_sensors(){ // Gather raw IR values and update nominal values accordingly
	uint16_t sum_FL = 0;
	uint16_t sum_FR = 0;
	uint16_t sum_L = 0;
	uint16_t sum_R = 0;

	unsigned num_polls = 1;
	for (num_polls; num_polls <= 3; num_polls++) {
		sum_FL += measure_dist(DIST_FL);
		sum_FR += measure_dist(DIST_FR);
		sum_L += measure_dist(DIST_L);
		sum_R += measure_dist(DIST_R);
	}
	raw_FL = sum_FL / num_polls;
	raw_FR = sum_FR / num_polls;
	raw_L = sum_L / num_polls;
	raw_R = sum_R / num_polls;

	nom_FL = raw_FL * scale_FL;
	nom_FR = raw_FR * scale_FR;
	nom_L = raw_L * scale_L;
	nom_R = raw_R * scale_R;
}

bool wallFront() { // Check if wall in front
	poll_sensors();
	float front_avg = (nom_FL + nom_FR) / 2;
	if (front_avg > 120) { // front_avg < 230 &&
		wall_front = true;
		HAL_GPIO_WritePin(LED_Red_GPIO_Port, LED_Red_Pin, 0);
	}
	else {
		wall_front = false;
		HAL_GPIO_WritePin(LED_Red_GPIO_Port, LED_Red_Pin, 1);
	}
	return wall_front;
}

bool wallRight() { // Check if wall to the right
	poll_sensors();
	if (nom_R > 20) { // nom_R > 20
		wall_right = true;
		HAL_GPIO_WritePin(LED_Green_GPIO_Port, LED_Green_Pin, 0);
	}
	else {
		wall_right = false;
		HAL_GPIO_WritePin(LED_Green_GPIO_Port, LED_Green_Pin, 1);
	}
	return wall_right;
}

bool wallLeft() { // Check if wall to the left
	poll_sensors();
	if (nom_L > 10) { // nom_L > 10
		wall_left = true;
		HAL_GPIO_WritePin(LED_Blue_GPIO_Port, LED_Blue_Pin, 0);
	}
	else {
		wall_left = false;
		HAL_GPIO_WritePin(LED_Blue_GPIO_Port, LED_Blue_Pin, 1);
	}
	return wall_left;
}
