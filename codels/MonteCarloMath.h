/*
 * MonteCarloMath.h
 *
 *  Created on: Nov 23, 2009
 *      Author: acodas
 */

#ifndef MONTECARLOMATH_H_
#define MONTECARLOMATH_H_
#include "MonteCarloFunctions.h"
double mc_getRandomUniformDouble() ;
double mc_getRandomUniformDouble(double min, double max) ;
double mc_getGaussDouble(double m, double e);
void mc_generateGaussianRandomPoint(mc_Point center,
		double covariance[3][3],mc_Point *newPoint);
void mc_generateGaussianRandomPoint(double distributionPoint[3],const double covariance[3][3]);
void mc_getCholeskyDecomposition(const double A[3][3], double L[3][3]) ;
void mc_multiplyMatrices(const double A[3][3],const  double B[3][3], double P[3][3]);
void mc_multiplyMatrices(const double A[3][3],const  double B[3], double P[3]);
void mc_substractMatrices(const double A[3][3],const  double B[3][3], double S[3][3]) ;
void mc_transposeMatrix(const double A[3][3], double P[3][3]) ;
void mc_substractVector(const double A[3],const double B[3], double S[3]);
void mc_sumVector(const double A[3], const double B[3], double S[3]);
void mc_rotation(double point[2],double theta);
#endif /* MONTECARLOMATH_H_ */
