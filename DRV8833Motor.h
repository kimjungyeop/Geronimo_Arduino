/*
 * DRV8833Motor.h
 *
 *  Created on: Nov 16, 2016
 *      Author: farandhigh
 */

#ifndef DRV8833MOTOR_H
#define DRV8833MOTOR_H

#include <Arduino.h>
#include "PololuQuadratureEncoder.h"

class DRV8833Motor{
public:
	DRV8833Motor(int out1, int out2, PololuQuadratureEncoder *ptr, float gearRatio, int mode);
	void setKValue(float Kp, float Ki, float Kd);
	void init();
	void set(float speed, int dt);
	void setPos(float pos, int dt);
	void run(float power);
	void PIDcontrol(float dt);
	void reverse();
	void resetTacho();
	long readTacho();
	void flipDecay();
	float getPower();

private:
	PololuQuadratureEncoder *encoder;
	unsigned const int out1, out2;
	float speed = 0;
	float iTerm = 0;
	float position = 0;
	bool positionControl = false;
	float power = 0;
	float gearRatio;
	float ticksPerRotation;
	int encoderMode;
	long prevTacho = 0;
	float integral = 0;
	float error_prior = 0;
	float error_prior_prior = 0;
	float output_prior = 0;
	float Kp = 0;
	float Ki = 0;
	float Kd = 0;
	bool initialized = false;
	bool flip = false;
	bool decay = false;
};
#endif /* DRV8833MOTOR_H_ */
