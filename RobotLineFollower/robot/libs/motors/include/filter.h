/*
 * filter.h
 *
 *  Created on: Mar 11, 2019
 *      Author: dangield
 */

#ifndef LSM6DS3_FILTER_H_
#define LSM6DS3_FILTER_H_

class filter {
public:
	filter(float, float, float);
	float getAngle(float, float);
	float getGyro();
	void setFactor(float);
	void setFrequency(float);
private:
	float previousAngles[3];
	float filteredAngle;
	float filteredGyro;
	float factor;
	float freq;
};



#endif /* LSM6DS3_FILTER_H_ */
