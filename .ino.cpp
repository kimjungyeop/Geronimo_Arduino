#ifdef __IN_ECLIPSE__
//This is a automatic generated file
//Please do not modify this file
//If you touch this file your change will be overwritten during the next build
//This file has been generated on 2017-04-26 13:29:43

#include "Arduino.h"
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
#include <algorithm>
#include <vector>
void initSensors() ;
void setup() ;
void resetSteadyState() ;
void turn(bool isLeft) ;
void front() ;
void gyroUpdate(float dt) ;
void straightUntilWall() ;
int getWallStatus() ;
bool isWall(float value) ;
void loop() ;

#include "main.ino"


#endif
