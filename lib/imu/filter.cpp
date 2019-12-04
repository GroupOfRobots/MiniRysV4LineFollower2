/*
 * filter.cpp
 *
 *  Created on: Mar 11, 2019
 *      Author: dangield
 */

#include "filter.h"

filter::filter(float angle, float x = 1, float hz = 100){
	previousAngles[0] = angle;
	previousAngles[1] = angle;
	previousAngles[2] = angle;
	filteredAngle = angle;
	filteredGyro = 0;
	freq = hz;
	if (x <= 1 and x >= 0) factor = x;
	else factor = 1;
}

float filter::getAngle(float angle, float gyro){
	float newAngle;
	newAngle = (filteredAngle + gyro/freq)*(1-factor)+factor*(angle+previousAngles[0]+previousAngles[1]+previousAngles[2])/4;
	filteredGyro = (newAngle - filteredAngle)*freq;
	filteredAngle = newAngle;
	previousAngles[2] = previousAngles[1];
	previousAngles[1] = previousAngles[0];
	previousAngles[0] = angle;
	return newAngle;
}

float filter::getGyro(){
	return filteredGyro;
}

void filter::setFactor(float x){
	if (x <= 1 and x >= 0) factor = x;
	else factor = 1;
}

void filter::setFrequency(float hz){
	freq = hz;
}
