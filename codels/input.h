/*
 * input.h
 *
 *  Created on: 30 nov. 2009
 *      Author: andres
 */

#ifndef INPUT_H_
#define INPUT_H_

#include "TagPositions.h"



int initInput();


int readRflex(double odo_position[3], double odo_cov[3][3]);

int readRFID( TagDetection** tagDetections);

#endif /* INPUT_H_ */
