/*
 * event_recorder.c
 *
 *  Created on: Nov 27, 2021
 *      Author: rafal
 */
#include "event_recorder.h"
#include <string.h>
#include "fatfs.h"
#include  <math.h>

static const char* file_name = "data.csv";
static const uint32_t WsPerkWh = 3600000;
static uint32_t row_index = 0;

static void initSDcard() {
	BYTE opt = 1;
	int status = f_mount(&SDFatFS, SDPath, opt);
	status = f_open(&SDFile, file_name, FA_WRITE | FA_CREATE_NEW);
	if (FR_OK == status) {
		unsigned int wbytes;
		const char* init_row = "Index, name, event time, duration, avr. I [A], Energy [kWh]\n";
		f_write(&SDFile, init_row, strlen(init_row), &wbytes);
		f_close(&SDFile);
	}
}

static void itoFractional(uint32_t num, uint32_t precision_positions, char* buf) {
	if (num < pow(10, precision_positions)) {
		char* numBuf = alloca(precision_positions+1);
		itoa(num, numBuf, 10);
		uint32_t refill = precision_positions - strlen(numBuf);
		for (uint32_t i=0; i<refill; i++ ){
			strcat(buf, "0");
		}
		strcat(buf, numBuf);
	}
}

static char* ftoa(float num, char* buf) {
	int numFicedPoint = num*1000;
	char numBuf[12] = {0};
	itoa(numFicedPoint/1000, numBuf, 10);
	strcat(buf, "\"");
	strcat(buf, numBuf); strcat(buf, ",");
	memset(numBuf, 0, sizeof(numBuf));
	itoFractional(numFicedPoint%1000, 3, numBuf);
	strcat(buf, numBuf);
	strcat(buf, "\"");
	return buf;
}

static char* convertTime(uint32_t time_ms, char* buf) {
	uint32_t ms = time_ms%1000;
	time_ms /= 1000;
	uint32_t s = time_ms%60;
	time_ms /= 60;
	uint32_t m = time_ms%60;
	time_ms /= 60;
	uint32_t h = time_ms;
	char numBuf[9] = {0};
	itoa(h, numBuf, 10);
	strcat(buf, "\"");
	strcat(buf, numBuf); strcat(buf, ":"); memset(numBuf, 0, sizeof(numBuf));
	itoa(m, numBuf, 10);
	strcat(buf, numBuf); strcat(buf, ":"); memset(numBuf, 0, sizeof(numBuf));
	itoa(s, numBuf, 10);
	strcat(buf, numBuf); strcat(buf, ","); memset(numBuf, 0, sizeof(numBuf));
	itoFractional(ms, 3, numBuf);
	strcat(buf, numBuf);
	strcat(buf, "\"");
	return buf;
}

static void recordData(const char* name, uint32_t start_time, uint32_t duration, float averageCurrent, float energy) {
	unsigned int wbytes;
	BYTE opt = 1;
	int status = f_mount(&SDFatFS, SDPath, opt);
	status = f_open(&SDFile, file_name, FA_WRITE | FA_OPEN_APPEND);

	if (FR_OK == status) {
		char strBuf[128] = {0};
		char tempBuf[24] = {0};
		itoa(row_index++, tempBuf, 10);
		strcat(strBuf, tempBuf);
		strcat(strBuf, ", ");
		strcat(strBuf, name);
		strcat(strBuf, ", ");
		memset(tempBuf, 0, sizeof(tempBuf));
		strcat(strBuf, convertTime(start_time, tempBuf));
		strcat(strBuf, ", ");
		memset(tempBuf, 0, sizeof(tempBuf));
		strcat(strBuf, convertTime(duration, tempBuf));
		strcat(strBuf, ", ");
		memset(tempBuf, 0, sizeof(tempBuf));
		strcat(strBuf, ftoa(averageCurrent, tempBuf));
		strcat(strBuf, ", ");
		memset(tempBuf, 0, sizeof(tempBuf));
		strcat(strBuf, ftoa(energy/WsPerkWh, tempBuf));
		strcat(strBuf, "\n");
		f_write(&SDFile, strBuf, strlen(strBuf), &wbytes);
		f_close(&SDFile);
	}
}

void initRecorder(event_recorder* recorder, const char* name) {
	recorder->name = (char*)malloc(strlen(name));
	strcpy(recorder->name, name);
	initSDcard();
}

void deleteRecorder(event_recorder* recorder) {
	free(recorder->name);
	recorder->start_time = 0;
}

void startEvent(event_recorder* recorder) {
	recorder->start_time = HAL_GetTick();
}

void endEvent(event_recorder* recorder, float averageCurrent, float energy) {
	uint32_t end_time = HAL_GetTick();
	uint32_t duration = end_time - recorder->start_time;
	recordData(recorder->name, recorder->start_time, duration, averageCurrent, energy);
}
