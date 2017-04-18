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
	DRV8833Motor(int out1, int out2, PololuQuadratureEncoder *ptr, double gearRatio, int mode);
	void setKValue(double Kp, double Ki, double Kd);
	void init();
	void set(double speed);
	void run(double power);
	void PIDcontrol(int dt);
	void reverse();
	void resetTacho();
	long readTacho();
	void flipDecay();
	double getPower();

private:
	PololuQuadratureEncoder *encoder;
	unsigned const int out1, out2;
	volatile double speed = 0;
	volatile double power = 0;
	double gearRatio;
	double ticksPerRotation;
	int encoderMode;
	volatile long prevTacho = 0;
	volatile double integral = 0;
	volatile double error_prior = 0;
	volatile double error_prior_prior = 0;
	volatile double output_prior = 0;
	double Kp = 0;
	double Ki = 0;
	double Kd = 0;
	bool initialized = false;
	bool flip = false;
	bool decay = false;
};
#endif /* DRV8833MOTOR_H_ */
