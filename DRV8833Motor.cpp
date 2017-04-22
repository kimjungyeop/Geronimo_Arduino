/*
 * DRV8833Motor.cpp
 *
 *  Created on: Nov 16, 2016
 *      Author: farandhigh
 */

#include <Arduino.h>
#include "DRV8833Motor.h"

DRV8833Motor::DRV8833Motor(int out1, int out2, PololuQuadratureEncoder *ptr,
		float gearRatio, int mode) :
		out1(out1), out2(out2) {
	this->gearRatio = gearRatio;
	this->encoder = ptr;
	this->encoderMode = mode;
	this->speed = 0;
	this->power = 0;
	this->ticksPerRotation = gearRatio * (encoder->ticksPerRotation);
}

void DRV8833Motor::setKValue(float Kp, float Ki, float Kd) {
	this->Kp = Kp;
	this->Ki = Ki;
	this->Kd = Kd;
}

void DRV8833Motor::init() {
	pinMode(out1, OUTPUT);
	pinMode(out2, OUTPUT);
	digitalWrite(out1, LOW);
	digitalWrite(out2, LOW);
	//Serial.println("MOTOR SET");
	speed = 0;
}

long DRV8833Motor::readTacho() {
	return encoder->getTacho(encoderMode);
}

void DRV8833Motor::run(float power) {
	// Run Motor with Power -255 to 255
	pinMode(out1, OUTPUT);
	pinMode(out2, OUTPUT);
	power = (int)power;
	this->power = power;
	if (flip) {
		power = -power;
	}

	if (power > 0) {
		if (decay) {
			analogWrite(out1, power);
			digitalWrite(out2, HIGH);
		} else {
			//Serial.print(power);
			//Serial.println("- 1");
			digitalWrite(out1, LOW);
			analogWrite(out2, power);
		}
	} else {
		if (decay) {
			digitalWrite(out1, HIGH);
			analogWrite(out2, -power);
		} else {
			//Serial.print(power);
			//Serial.println("- 2");
			analogWrite(out1, -power);
			digitalWrite(out2, LOW);
		}
	}
}

void DRV8833Motor::set(float speed, int dt) {
	// Run Motor with RPS specified in speed
	error_prior = error_prior - (speed - this->speed);
	error_prior_prior = error_prior_prior - (speed - this->speed);
	this->speed = speed;
	PIDcontrol(dt);

	/*if (speed == 0) {
		//error_prior = 0;
		//error_prior_prior = 0;
		//output_prior = 0;
		power = 0;
		//integral = 0;
		this->speed = 0;
		run(0);
	}
	else {
		//error_prior_prior -= (latestSpeed - this->speed);
		//output_prior = 0;
		//error_prior -= (latestSpeed - this->speed);
		//power = 0;
		//integral = 0;
		this->speed = speed;
	}*/

}

float DRV8833Motor::getPower() {
	return power;
}

void DRV8833Motor::PIDcontrol(float dt) {
	// speed control for
	long curTacho = encoder->getTacho(encoderMode);
	float curSpeed = (curTacho - prevTacho) / dt * 1000.0 / ticksPerRotation; // Rounds Per Second
	float error = speed - curSpeed;
	float output = output_prior - Kp * (error - error_prior) - Ki * error
			- Kd * (error - 2 * error_prior + error_prior_prior);

	if (speed == 0 && curSpeed < 0.1 && curSpeed > -0.1){
		output = 0;
	}
	else if (output > 200) {
		output = 200;
	}
	else if (output < -200) {
		output = -200;
	}
	run(output);
	output_prior = output;

	prevTacho = curTacho;

	error_prior_prior = error_prior;
	error_prior = error;

	//Serial.println("-----");
	//Serial.println(error);
	//Serial.println(curSpeed);
	//Serial.println(output);

	// u(t) = u(t-1) + K_p * (e(t) - e(t-1)) + K_i *T_s * e(t)
	//u ( t + 1) = u ( t ) − kIe(t) − kP(e (t) − e (t- 1) )
	// − kD (e(t) − 2e(t −1) + e(t − 2))
	// u(t) = u(t-1) + K_p * (e(t) - e(t-1)) + K_i *T_s * e(t)
}

void DRV8833Motor::resetTacho() {
	encoder->reset(encoderMode);
}

void DRV8833Motor::reverse() {
	flip = !flip;
}

void DRV8833Motor::flipDecay() {
	decay = !decay;
}
