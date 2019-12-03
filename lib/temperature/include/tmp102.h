/*
 * tmp102.h
 *
 *  Created on: 15 gru 2018
 *      Author: kamil
 */

#ifndef TMP102_H_
#define TMP102_H_

#include <errno.h>
#include <string.h>
#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>
#include <linux/i2c-dev.h>
#include <sys/ioctl.h>
#include <sys/types.h>
#include <sys/stat.h>
#include <fcntl.h>
#include <stdint.h>
#include "../include/gpio.h"


#define DEFAULT_I2C_BUS		"/dev/i2c-1"


class tmp102 {
public:
	tmp102(uint8_t addr, char* i2c_bus);
    float readTemperature();
private:
    int file;
	char filename[40];
	GPIO* gpio4;
};


#endif /* TMP102_H_ */
