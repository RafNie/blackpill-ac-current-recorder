/*
 * input_minitor.c
 *
 *  Created on: Nov 27, 2021
 *      Author: rafal
 */

#include "input_monitor.h"

static inline float calculateCurrent(float pp_voltage) {
	return pp_voltage/(currentTransformerRatio*2);
}

static inline void clearMonitor(input_monitor* input_monitor) {
	input_monitor->current_sum = 0;
	input_monitor->sample_counter = 0;
	input_monitor->energy = 0;
}


void countInput(input_monitor* input_monitor, float pp_voltage) {
	float current = calculateCurrent(pp_voltage);
	if (current > currentThreshold) {
		HAL_GPIO_WritePin(led_pin_GPIO_Port, led_pin_Pin, GPIO_PIN_RESET);
		input_monitor->current_sum += current;
		input_monitor->sample_counter++;
		input_monitor->energy += powerVoltage*current;
		if (input_monitor->sample_counter == 1 &&
				input_monitor->on_detected_event) {
			input_monitor->on_detected_event();
			HAL_GPIO_WritePin(led_pin_GPIO_Port, led_pin_Pin, GPIO_PIN_RESET);
		}
	} else {
		if (input_monitor->sample_counter > 0 &&
				input_monitor->off_detected_event) {
			float avgCurrent = input_monitor->current_sum/input_monitor->sample_counter;
			input_monitor->off_detected_event(avgCurrent, input_monitor->energy);
			clearMonitor(input_monitor);
			HAL_GPIO_WritePin(led_pin_GPIO_Port, led_pin_Pin, GPIO_PIN_SET);
		}
	}
}
