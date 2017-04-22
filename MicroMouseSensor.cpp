/*
 * MicroMouseSensor.cpp
 *
 *  Created on: Jan 27, 2017
 *      Author: farandhigh
 */
#include <Arduino.h>
#include "MicroMouseSensor.h"
#include <algorithm>    // std::sort
#include <vector>       // std::vector

MicroMouseSensor::MicroMouseSensor(int frontIR, int leftIR, int rightIR) :
IR_F(frontIR), IR_L(leftIR), IR_R(rightIR) {
	pinMode(IR_F, INPUT);
	pinMode(IR_L, INPUT);
	pinMode(IR_R, INPUT);
}

float MicroMouseSensor::getMedian(float data[]) {
	int size = sizeof(data)/sizeof(float);
	std::sort(&data[0], &data[size]);
	return size % 2 ? data[size / 2] : (data[size / 2 - 1] + data[size / 2]) / 2;
}

int MicroMouseSensor::readLeftIR() {
	//int data[6];
	int sum = 0;
	for(int i = 0; i < 6; i++) {
		//data[i] = analogRead(IR_L);
		sum += analogRead(IR_L);
	}
	//int size = sizeof(data)/sizeof(int);
	//std::sort(&data[0], &data[size]);
	return sum / 6;
			//size % 2 ? data[size / 2] : (data[size / 2 - 1] + data[size / 2]) / 2;
	//return analogRead(IR_L);
	/*
	int ret = 130 - analogRead(IR_L);
	if (ret >= 0) {
		return analogRead(IR_L);
	}
	else {
		return 0;
	}*/
}

int MicroMouseSensor::readRightIR() {
	//int data[6];
	int sum = 0;
	for(int i = 0; i < 6; i++) {
		//data[i] = analogRead(IR_L);
		sum += analogRead(IR_R);
	}
	//int size = sizeof(data)/sizeof(int);
	//std::sort(&data[0], &data[size]);
	return sum / 6;
	//return size % 2 ? data[size / 2] : (data[size / 2 - 1] + data[size / 2]) / 2;
	//return analogRead(IR_R);
	/*
	int ret = 130 - analogRead(IR_R);
	if (ret >= 0) {
		return analogRead(IR_R);
	}
	else {
		return 0;
	}*/
}

int MicroMouseSensor::readFrontIR() {
	//int data[6];
	int sum = 0;
	for(int i = 0; i < 6; i++) {
		//data[i] = analogRead(IR_L);
		sum += analogRead(IR_F);
	}
	//int size = sizeof(data)/sizeof(int);
	//std::sort(&data[0], &data[size]);
	return sum / 6;
	//return size % 2 ? data[size / 2] : (data[size / 2 - 1] + data[size / 2]) / 2;
	//return analogRead(IR_F);
	/*int ret = 130 - analogRead(IR_F);
	if (ret >= 0) {
		return analogRead(IR_F);
	}
	else {
		return 0;
	}*/
}
