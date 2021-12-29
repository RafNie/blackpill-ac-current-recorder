/*
 * input_monitor.c
 *
 *  Created on: Nov 27, 2021
 *      Author: rafal
 */

#ifndef INC_INPUT_MONITOR_H_
#define INC_INPUT_MONITOR_H_

#include <inttypes.h>
#include <main.h>

static const float currentThreshold = 0.6;
static const float powerVoltage = 230;
static const float currentTransformerRatio = 0.08132;
static const float rootOfTwo = 1.4142135623731;

typedef struct {
	float energy;//[Ws]
	uint32_t sample_counter;
	float current_sum; //[A]
	void (*on_detected_event)();
	void (*off_detected_event)(float averageCurrent, float energy);
} input_monitor;

void countInput(input_monitor* input_monitor, float pp_voltage);

#endif /* INC_INPUT_MONITOR_H_ */
