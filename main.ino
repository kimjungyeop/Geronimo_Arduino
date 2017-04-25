#define M1PWM1 20
#define M1PWM2 21
#define M2PWM1 23
#define M2PWM2 22
#define M1TACHO1 2
#define M1TACHO2 3
#define M2TACHO1 1
#define M2TACHO2 0
#define IRFRONT 15
#define IRLEFT 16
#define IRRIGHT 17

#include <Arduino.h>
#include <Wire.h>
#include "DRV8833Motor.h"
#include "PololuQuadratureEncoder.h"
#include "MicroMouseSensor.h"
#include <Adafruit_Sensor.h>
#include <Adafruit_LSM303_U.h>
#include <Adafruit_L3GD20_U.h>
#include <Adafruit_9DOF.h>
#include <algorithm>    // std::sort
#include <vector>       // std::vector
using namespace std;

DRV8833Motor *motorL;
DRV8833Motor *motorR;
PololuQuadratureEncoder *tacho;
MicroMouseSensor *sensor;
int dt = 5000; // 5microsec control loop
Adafruit_9DOF                 dof   = Adafruit_9DOF();
Adafruit_LSM303_Accel_Unified accel = Adafruit_LSM303_Accel_Unified(30301);
Adafruit_LSM303_Mag_Unified   mag   = Adafruit_LSM303_Mag_Unified(30302);
Adafruit_L3GD20_Unified gyro = Adafruit_L3GD20_Unified(20);

float angle = 0;
float steadyState = 0;
float millis_prev = 0;
float timeSinceCalibration = 0;
float speedL = 0;
float speedR = 0;
bool finishedTurning = false;

void initSensors() {
  if(!accel.begin()) {/* There was a problem detecting the LSM303 ... check your connections */
    Serial.println(F("Ooops, no LSM303 detected ... Check your wiring!"));
    while(1);
  }
  if(!mag.begin()) {
    /* There was a problem detecting the LSM303 ... check your connections */
    Serial.println("Ooops, no LSM303 detected ... Check your wiring!");
    while(1);
  }
  if(!gyro.begin()) {
    /* There was a problem detecting the L3GD20 ... check your connections */
    Serial.println("Ooops, no L3GD20 detected ... Check your wiring!");
    while(1);
  }
}

void setup() {
	// sensor initialization
	Serial.begin(9600);
	analogReadResolution(8);
	initSensors();
	sensor = new MicroMouseSensor(IRFRONT, IRLEFT, IRRIGHT);

	// motor initialization
	tacho = new PololuQuadratureEncoder(M1TACHO1, M1TACHO2, M2TACHO1, M2TACHO2, 12);
	tacho->init();
	motorL = new DRV8833Motor(M1PWM1, M1PWM2, tacho, 100.37, 0);
	motorR = new DRV8833Motor(M2PWM1, M2PWM2, tacho, 100.37, 1);
	motorL->init();
	motorL->setKValue(10, 1.1, 0.5);
	motorR->init();
	motorR->setKValue(10, 1.1, 0.5);

	// led initialization
	pinMode(13, OUTPUT);
	for (int i = 0; i < 10; i++) {
		digitalWrite(13, HIGH);
		delay(100);
		digitalWrite(13, LOW);
		delay(100);
	}
	digitalWrite(13, HIGH);
	delay(1000);

	// gyro calibration
	float data[20];
	float sum = 0;
	for(int i = 0; i < 20; i++) {
		sensors_event_t event;
		gyro.getEvent(&event);
		data[i] = event.gyro.z;
		sum += data[i];
		delay(10);
	}
	steadyState = sum / 20;
	digitalWrite(13, LOW);
	timeSinceCalibration = millis();
}

void resetSteadyState() {
	digitalWrite(13, HIGH);
	float data[10];
	float sum = 0;
	for(int i = 0; i < 10; i++) {
		sensors_event_t event;
		gyro.getEvent(&event);
		data[i] = event.gyro.z;
		sum += data[i];
	}
	steadyState = sum / 10;
	digitalWrite(13, LOW);
}


void turn(bool isLeft) {
	finishedTurning = false;
	if (isLeft) {
		motorL->setPos(904, 0);
		motorR->setPos(-905, 0);
	}
	else {
		motorL->setPos(-904, 0);
		motorR->setPos(905, 0);
	}

	while(motorL->positionControl && motorR->positionControl) {
		motorL->PIDcontrol(0);
		motorR->PIDcontrol(0);
		delay(10);
	}
	finishedTurning = true;
	delay(500);
	resetSteadyState();
	delay(500);
}

void gyroUpdate(float dt) {
	sensors_event_t gyro_event;
	gyro.getEvent(&gyro_event);
	angle = angle + (gyro_event.gyro.z - steadyState) * 57.295779513 * dt / 1000;
}

void straightUntilWall() {
    float dir = angle;
    float prevTime = millis();
    float kp = 0.005, ki = 0.00001, kd = 0.0001;
    float i = 0;
    float prevE = 0;

    delay(500);
	resetSteadyState();
    delay(500);

    while (sensor->readFrontIR() < 80 || sensor->readFrontIR() > 85) { // Change the distance value.
    	delay(10);
        float curTime = millis();
        float dt = curTime - prevTime;
        gyroUpdate(dt);
        Serial.println(angle);

        float diff = ((((int)angle % 360) - ((int)dir % 360)) + 720) % 360;
        // Diff is the change in orientation, as an angle between 0 and 360
		float e = (diff < 180) ? diff : diff - 360;
        // e is the error, an angle between -180 and 180

        i += (e * dt);
		float de = (e - prevE) / dt;
		prevE = e;

		double u = (kp * e) + (ki * i) + (kd * de);
		if (u > 2) {
			u = 2;
		}
		else if (u < -2) {
			u = -2;
		}
		motorL->set(1 - u, dt);
        motorR->set(1 + u, dt);
        prevTime = curTime;
    }

    float e = 10;
    while (abs(e) > 1) { // Change the distance value.
        delay(10);
        float curTime = millis();
        float dt = curTime - prevTime;

        float diff = ((((int)angle % 360) - ((int)dir % 360)) + 720) % 360;
        // Diff is the change in orientation, as an angle between 0 and 360
		e = (diff < 180) ? diff : diff - 360;
        // e is the error, an angle between -180 and 180

        i += (e * dt);
		float de = 0;
		if (prevE != -1000) {
			de = (e - prevE) / dt;
		}
		prevE = e;

		double u = (kp * e) + (ki * i) + (kd * de);
		if (u > 2) {
			u = 2;
		}
		else if (u < -2) {
			u = -2;
		}
        motorL->set(-u, dt);
        motorR->set(u, dt);
        gyroUpdate(dt);
        prevTime = curTime;
    }
    delay(500);
	resetSteadyState();
    delay(500);
}

void loop() {
	straightUntilWall();
	turn(true);
	/*Serial.println(sensor->readFrontIR());
	Serial.println(sensor->readLeftIR());
	Serial.println(sensor->readRightIR());
	Serial.println("---");
	delay(100);*/
	/*
	turn(true);
	turn(false);
	turn(true);
	turn(false);
	turn(true);
	turn(false);
	turn(true);
	turn(false);
	turn(true);
	turn(false);
	float curTime = millis();
	float dt = (curTime - millis_prev);

	// 1: Update Gyro
	gyroUpdate(dt);

	// 2: Speed Control

	millis_prev = curTime;

	int dly = 10 - (millis() - curTime);
	if (delay > 0) {
		delay(dly);
	}*/
}
