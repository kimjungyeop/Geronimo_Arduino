#define M1PWM1 21
#define M1PWM2 20
#define M2PWM1 22
#define M2PWM2 23
#define M1TACHO1 3
#define M1TACHO2 2
#define M2TACHO1 0
#define M2TACHO2 1
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
IntervalTimer imu;
IntervalTimer gyroscope;

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
	motorL = new DRV8833Motor(M1PWM1, M1PWM2, tacho, 100, 0);
	motorR = new DRV8833Motor(M2PWM1, M2PWM2, tacho, 100, 1);
	motorL->init();
	motorL->setKValue(10, 0, 0);
	motorR->init();
	motorR->setKValue(10, 0, 0);

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
		sum = sum + data[i];
		delay(100);
	}
	steadyState = sum / 20;
	digitalWrite(13, LOW);
	delay(1000);
}

float ComplementaryFilter(float accDataX,float accDataY,float accDataZ,float gyroZ, float dt, float previous_heading)
{
  /*This function gets accelerometer and gyroscope readings as an input and give a value for the pitch angle*/
  /*INPUT*/
  /*dt [s], acc [m/s^2], gyro [deg/s]*/
	float a = 1;
    //float signOfX = accDataX >= 0 ? -1.0F : 1.0F;
    float heading_fromAcc, t_heading, heading_fromGyro, heading_comp_filter;
    // Integrate the gyroscope data -> int(angularSpeed) = angle
    heading_fromGyro = previous_heading + (gyroZ - steadyState) * 57.295779513 * dt; //((float)gyroY) * dt; // Angle around the X-axis 
    // Compensate for drift with accelerometer data if !bullshit
    //t_heading= accDataX * accDataX + accDataY * accDataY;
    //heading_fromAcc = (float)atan2(accDataZ, signOfX * sqrt(t_heading)) * 180 / PI;
    heading_comp_filter = a * heading_fromGyro;
    		//+ (1 - a) * heading_fromAcc;
    return heading_comp_filter;
}

bool resetSteadyState() {
	digitalWrite(13, HIGH);
	float data[10];
	float sum = 0;
	for(int i = 0; i < 10; i++) {
		sensors_event_t event;
		gyro.getEvent(&event);
		data[i] = event.gyro.z;
		sum = sum + data[i];
		delay(15);
	}
	steadyState = sum / 10;
	digitalWrite(13, LOW);
	motorL->run(1);
	motorR->run(1);
}

void loop() {
	// 1. Gyro Value Calculation
	sensors_event_t gyro_event;
	gyro.getEvent(&gyro_event);

	float curTime = millis();
	float dt = (curTime - millis_prev);
	millis_prev = curTime;

	angle =  angle + (gyro_event.gyro.z - steadyState) * 57.295779513 * dt / 1000;

	// 2. Motor Speed Control
	motorL->PIDcontrol(dt);
	motorR->PIDcontrol(dt);

	// 3. Communications
	if (Serial.available()) {
		// L20 R40 LED1
		// 20 40 1
		String command = Serial.readString();
		command = command.substring(command.indexOf("S") + 1);
		int left = command.substring(0, command.indexOf("|")).toInt();
		String rest1 = command.substring(command.indexOf("|") + 1);
		int right = rest1.substring(0, rest1.indexOf("|")).toInt();
		String rest2 = rest1.substring(rest1.indexOf("|") + 1);
		rest2 = rest2.substring(0, rest2.indexOf("E"));
		int led = rest2.toInt();

		motorL->run(left);
		motorR->run(right);
		digitalWrite(13, led);

		if (led == 1) {
			resetSteadyState();
			digitalWrite(13, 0);
		}

		// L52 R102 F60 IMU10 LT1023 RT5034
		// 52 102 60 10 1023 5034

		Serial.println("l" + String(sensor->readLeftIR()));
		Serial.println("r" + String(sensor->readRightIR()));
		Serial.println("f" + String(sensor->readFrontIR()));
		Serial.println("i" + String(angle));
		Serial.println("tl" + String(motorL->readTacho()));
		Serial.println("tr" + String(motorR->readTacho()));
	}
	delay(5 - (millis() - curTime));
}
