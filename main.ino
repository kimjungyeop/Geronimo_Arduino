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
float speedL = 0;
float speedR = 0;

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

void loop() {
	if (Serial.available()) {
		// Communications
		// L2 R4 RESET1
		// S2|4|1E
		String command = Serial.readString();
		command = command.substring(command.indexOf("S") + 1);
		float left = command.substring(0, command.indexOf("|")).toFloat();
		String rest1 = command.substring(command.indexOf("|") + 1);
		float right = rest1.substring(0, rest1.indexOf("|")).toFloat();
		String rest2 = rest1.substring(rest1.indexOf("|") + 1);
		rest2 = rest2.substring(0, rest2.indexOf("E"));
		int reset = rest2.toInt();

		float curTime = millis();
		float dt = (curTime - millis_prev);
		sensors_event_t gyro_event;
		gyro.getEvent(&gyro_event);

		angle = angle + (gyro_event.gyro.z - steadyState) * 57.295779513 * dt / 1000;

		motorL->set(left, dt);
		motorR->set(right, dt);

		millis_prev = curTime;

		if (reset == 1) {
			resetSteadyState();
			digitalWrite(13, 1);
			millis_prev = millis();
		}

		// L52 R102 F60 IMU10 LT1023 RT5034
		// 52 102 60 10 1023 5034

		Serial.println("l" + String(sensor->readLeftIR()));
		Serial.println("r" + String(sensor->readRightIR()));
		Serial.println("f" + String(sensor->readFrontIR()));
		Serial.println("i" + String(angle));
		Serial.println("tl" + String(motorL->readTacho()));
		Serial.println("tr" + String(motorR->readTacho()));
		digitalWrite(13, 0);

		int dly = 10 - (millis() - curTime);
		if (delay > 0) {
			delay(dly);
		}
	}
	else {
		// 1. Gyro Value Calculation
		float curTime = millis();
		float dt = (curTime - millis_prev);

		sensors_event_t gyro_event;
		gyro.getEvent(&gyro_event);

		angle = angle + (gyro_event.gyro.z - steadyState) * 57.295779513 * dt / 1000;

		// 2. Motor Speed Control
		motorL->PIDcontrol(dt);
		motorR->PIDcontrol(dt);

		millis_prev = curTime;
		int dly = 10 - (millis() - curTime);
		if (delay > 0) {
			delay(dly);
		}
	}
}
