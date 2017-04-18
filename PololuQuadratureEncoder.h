/*
 * PololuQuadratureEncoder.h
 *
 *  Created on: Nov 16, 2016
 *      Author: farandhigh
 */

#ifndef POLOLUQUADRATUREENCODER_H_
#define POLOLUQUADRATUREENCODER_H_

#include <Arduino.h>

class PololuQuadratureEncoder {
public:
	PololuQuadratureEncoder(int t1, int t2);
	PololuQuadratureEncoder(int t11, int t12, int t21, int t22, int ticksPerRotation);
	static void init();
	static long getTacho(int mode);
	static void reset(int mode);
	static const int ticksPerRotation = 12;
private:
	static long tachoCount1;
	static long tachoCount2;
	static unsigned int tacho11;
	static unsigned int tacho12;
	static unsigned int tacho21;
	static unsigned int tacho22;
	static void doEncoder11();
	static void doEncoder12();
	static void doEncoder21();
	static void doEncoder22();
};

#endif /* POLOLUQUADRATUREENCODER_H_ */
