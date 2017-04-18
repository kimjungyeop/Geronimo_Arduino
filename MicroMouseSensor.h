/*
 * MicroMouseSensor.h
 *
 *  Created on: Dec 10, 2016
 *      Author: farandhigh
 */

#ifndef MICROMOUSESENSOR_H_
#define MICROMOUSESENSOR_H_

#include <Arduino.h>

class MicroMouseSensor {
public:
	MicroMouseSensor(int frontIR, int leftIR, int rightIR);
	int readLeftIR();
	int readRightIR(); //1 if object exists
	int readFrontIR(); //millimeters
	//double readIMU(); // degrees of rotation
private:
	unsigned const int IR_F, IR_L, IR_R;


};



#endif /* MICROMOUSESENSOR_H_ */
