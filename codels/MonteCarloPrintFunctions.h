/*
 * MonteCarloPrintFunctions.h
 *
 *  Created on: Nov 23, 2009
 *      Author: acodas
 */

#ifndef MONTECARLOPRINTFUNCTIONS_H_
#define MONTECARLOPRINTFUNCTIONS_H_

#include "MonteCarloFunctions.h"

void mc_printPoint(mc_Point p);

void mc_printSamples(const char OUTPUT_FILE_NAME[],const char fileOption[], const char* introduction, mc_Points currentPoints);

void mc_printMatrix(double A[3][3]) ;

void mc_printSamples(const char* introduction, mc_Points currentPoints);

void mc_printSamples(const char OUTPUT_FILE_NAME[],const char fileOption[], const char* introduction, mc_Points currentPoints);

void mc_printSamples(const char* introduction, mc_Points currentPoints) ;

void mc_printSamples(const char OUTPUT_FILE_NAME[],const char fileOption[], const char* introduction, mc_Points currentPoints);

void mc_printTagDetections(TagDetection* tagDetections);

void mc_printTagsExpected(TagExpectation* tagsExpected);

#endif /* MONTECARLOPRINTFUNCTIONS_H_ */
