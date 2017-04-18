/*
 * PololuQuadratureEncoder.cpp
 *
 *  Created on: Nov 16, 2016
 *      Author: farandhigh
 */

#include <Arduino.h>
#include "PololuQuadratureEncoder.h"

long PololuQuadratureEncoder::tachoCount1;
long PololuQuadratureEncoder::tachoCount2;
unsigned int PololuQuadratureEncoder::tacho11;
unsigned int PololuQuadratureEncoder::tacho12;
unsigned int PololuQuadratureEncoder::tacho21;
unsigned int PololuQuadratureEncoder::tacho22;

PololuQuadratureEncoder::PololuQuadratureEncoder(int t1, int t2) {
	tacho11 = t1;
	tacho12 = t2;
	tachoCount1 = 0;
}

PololuQuadratureEncoder::PololuQuadratureEncoder(int t1, int t2, int t3, int t4,
		int ticksPerRotation) {
	tacho11 = t1;
	tacho12 = t2;
	tacho21 = t3;
	tacho22 = t4;
	tachoCount1 = 0;
	tachoCount2 = 0;
}

void PololuQuadratureEncoder::init() {
	pinMode(tacho11, INPUT);
	pinMode(tacho12, INPUT);
	pinMode(tacho21, INPUT);
	pinMode(tacho22, INPUT);
	digitalWrite(tacho11, HIGH);
	digitalWrite(tacho12, HIGH);
	digitalWrite(tacho21, HIGH);
	digitalWrite(tacho22, HIGH);
	attachInterrupt(tacho11, doEncoder11, CHANGE);
	attachInterrupt(tacho12, doEncoder12, CHANGE);
	attachInterrupt(tacho21, doEncoder21, CHANGE);
	attachInterrupt(tacho22, doEncoder22, CHANGE);
	Serial.println("ENCODER SET");
}

long PololuQuadratureEncoder::getTacho(int mode) {
	if (mode == 0) {
		return tachoCount1;
	} else if (mode == 1) {
		return tachoCount2;
	} else {
		return -1;
	}
}

void PololuQuadratureEncoder::reset(int mode) {
	if (mode == 0) {
		tachoCount1 = 0;
	} else if (mode == 1) {
		tachoCount2 = 0;
	}

}

void PololuQuadratureEncoder::doEncoder11() {
	// look for a low-to-high on channel A
	if (digitalRead(tacho11) == HIGH) {
		// check channel B to see which way encoder is turning
		if (digitalRead(tacho12) == LOW) {
			tachoCount1 = tachoCount1 + 1;         // CW
		} else {
			tachoCount1 = tachoCount1 - 1;         // CCW
		}
	}
	// check channel B to see which way encoder is turning
	else { // must be a high-to-low edge on channel A
		if (digitalRead(tacho12) == HIGH) {
			tachoCount1 = tachoCount1 + 1;          // CW
		} else {
			tachoCount1 = tachoCount1 - 1;          // CCW
		}
	}
}

void PololuQuadratureEncoder::doEncoder12() {
	// look for a low-to-high on channel B
	if (digitalRead(tacho12) == HIGH) {
		// check channel A to see which way encoder is turning
		if (digitalRead(tacho11) == HIGH) {
			tachoCount1 = tachoCount1 + 1;         // CW
		} else {
			tachoCount1 = tachoCount1 - 1;         // CCW
		}
	}
	// Look for a high-to-low on channel B
	else {
		// check channel B to see which way encoder is turning
		if (digitalRead(tacho11) == LOW) {
			tachoCount1 = tachoCount1 + 1;          // CW
		} else {
			tachoCount1 = tachoCount1 - 1;          // CCW
		}
	}
}

void PololuQuadratureEncoder::doEncoder21() {
	// look for a low-to-high on channel A
	if (digitalRead(tacho21) == HIGH) {
		// check channel B to see which way encoder is turning
		if (digitalRead(tacho22) == LOW) {
			tachoCount2 = tachoCount2 + 1;         // CW
		} else {
			tachoCount2 = tachoCount2 - 1;         // CCW
		}
	}
	// check channel B to see which way encoder is turning
	else { // must be a high-to-low edge on channel A
		if (digitalRead(tacho22) == HIGH) {
			tachoCount2 = tachoCount2 + 1;          // CW
		} else {
			tachoCount2 = tachoCount2 - 1;          // CCW
		}
	}
}

void PololuQuadratureEncoder::doEncoder22() {
	// look for a low-to-high on channel B
	if (digitalRead(tacho22) == HIGH) {
		// check channel A to see which way encoder is turning
		if (digitalRead(tacho21) == HIGH) {
			tachoCount2 = tachoCount2 + 1;         // CW
		} else {
			tachoCount2 = tachoCount2 - 1;         // CCW
		}
	}
	// Look for a high-to-low on channel B
	else {
		// check channel B to see which way encoder is turning
		if (digitalRead(tacho21) == LOW) {
			tachoCount2 = tachoCount2 + 1;          // CW
		} else {
			tachoCount2 = tachoCount2 - 1;          // CCW
		}
	}
}
