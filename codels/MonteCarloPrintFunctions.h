/*
 * MonteCarloPrintFunctions.h
 *
 *  Created on: Nov 23, 2009
 *      Author: acodas
 */

#ifndef MONTECARLOPRINTFUNCTIONS_H_
#define MONTECARLOPRINTFUNCTIONS_H_

#include "MonteCarloFunctions.h"

void mc_printPoint(const mc_Point p);

int mc_printSamples(const char OUTPUT_FILE_NAME[],const  mc_Points *currentPoints);

void mc_printMatrix(const double A[3][3]) ;

//void mc_printSamples(const char* introduction, mc_Points currentPoints);

void mc_printTagDetections(const TagDetection* tagDetections);

void mc_printTagsExpected(const TagExpectation* tagsExpected);

#endif /* MONTECARLOPRINTFUNCTIONS_H_ */
