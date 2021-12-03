/*
 * event_recorder.h
 *
 *  Created on: Nov 27, 2021
 *      Author: rafal
 */

#ifndef INC_EVENT_RECORDER_H_
#define INC_EVENT_RECORDER_H_
#include <inttypes.h>

typedef struct {
	char* name;
	uint32_t start_time;
} event_recorder;

void initRecorder(event_recorder* recorder, const char* name);
void deleteRecorder(event_recorder* recorder);
void startEvent(event_recorder* recorder);
void endEvent(event_recorder* recorder, float averageCurrent, float energy);

#endif /* INC_EVENT_RECORDER_H_ */
