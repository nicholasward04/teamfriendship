/*
 * Module for the implementation my mouse' IR sensors and their normalization
 */
#ifndef __IR_H
#define __IR_H
#include "main.h"

typedef enum { // Enumeration for IR cases
	DIST_FL,
	DIST_FR,
	DIST_R,
	DIST_L
} dist_t;

void ADC1_Select_CH5(void);
void ADC1_Select_CH8(void);
void ADC1_Select_CH4(void);
void ADC1_Select_CH9(void);

unsigned measure_dist(dist_t dist);
void poll_sensors();
void pdScan();

bool wallFront();
bool wallRight();
bool wallLeft();

#endif
