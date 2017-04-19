#ifdef __IN_ECLIPSE__
//This is a automatic generated file
//Please do not modify this file
//If you touch this file your change will be overwritten during the next build
//This file has been generated on 2017-04-18 21:16:44

#include "Arduino.h"
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
#include <algorithm>
#include <vector>
void initSensors() ;
void setup() ;
bool resetSteadyState() ;
void loop() ;

#include "main.ino"


#endif
