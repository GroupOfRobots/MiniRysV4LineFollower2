/*
 * gpio.h
 *
 *  Created on: 27 gru 2018
 *      Author: kamil
 */

#ifndef GPIO_H_
#define GPIO_H_

#include <string>
using namespace std;

class GPIO
{
public:
    GPIO();  // create a GPIO object that controls GPIO4 (default
    GPIO(string x); // create a GPIO object that controls GPIOx, where x is passed to this constructor
    int export_gpio(); // exports GPIO
    int unexport_gpio(); // unexport GPIO
    int setdir_gpio(string dir); // Set GPIO Direction
    int setval_gpio(string val); // Set GPIO Value (putput pins)
    int getval_gpio(string& val); // Get GPIO Value (input/ output pins)
    string get_gpionum(); // return the GPIO number associated with the instance of an object
private:
    string gpionum; // GPIO number associated with the instance of an object
};


#endif /* GPIO_H_ */
