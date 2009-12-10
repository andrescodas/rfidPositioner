/*
 * MonteCarloMath.cc
 *
 *  Created on: Nov 23, 2009
 *      Author: acodas
 */
#include "MonteCarloMath.h"
#include <stdlib.h>


//@tested
//returns a uniformly distributed double between 0 and 1
double mc_getRandomUniformDouble() {
	return ((double) rand()) / (((double) RAND_MAX));
}
//returns a uniformly distributed double between min and max
double mc_getRandomUniformDouble(double min, double max) {
	return min + (max - min) * mc_getRandomUniformDouble();
}

//@tested
//Box-Muller algorithm
//returns a gaussianly distributed double, average=m, spread=e
double mc_getGaussDouble(double m, double e) {
	double a;
	double b;
	double s;
	do {
		a = mc_getRandomUniformDouble(-1.0, 1.0);
		b = mc_getRandomUniformDouble(-1.0, 1.0);
		s = a * a + b * b;
	} while ((s >= 1.0) or (s == 0.0));

	double x(sqrt(-2.0 * log(s) / s));
	if (mc_getRandomUniformDouble(0.0, 1.0) > 0.5) {
		x *= a;
	} else {
		x *= b;
	}

	return m + e * x;
}


//@tested
//draws a random point in the gaussian distribution centered in center and given by the covariance
mc_Point mc_generateGaussianRandomPoint(mc_Point center,
		double covariance[3][3]) {
	double gaussianVector[3] = { mc_getGaussDouble(0, 1), mc_getGaussDouble(0,
			1), mc_getGaussDouble(0, 1) };
	double L[3][3];
	mc_getCholeskyDecomposition(covariance, L);
	double vector[3];
	mc_multiplyMatrices(L, gaussianVector, vector);
	return mc_Point(center.x + vector[0], center.y + vector[1], center.theta
			+ vector[2]);
}


//@tested
//draws a random point in the gaussian distribution centered in center and given by the covariance
void mc_generateGaussianRandomPoint(double distributionPoint[3],double covariance[3][3]) {
	double gaussianVector[3] = { mc_getGaussDouble(0, 1), mc_getGaussDouble(0,1), mc_getGaussDouble(0, 1) };
	double L[3][3];

	mc_getCholeskyDecomposition(covariance, L);

	mc_multiplyMatrices(L, gaussianVector, distributionPoint);
}


//@tested
//returns the cholesky decomposition of A in L
void mc_getCholeskyDecomposition(double A[3][3], double L[3][3]) {
	L[0][0] = sqrt(A[0][0]);
	L[1][0] = 1 / L[0][0] * (A[1][0]);
	L[1][1] = sqrt(A[1][1] - (L[1][0] * L[1][0]));
	L[2][0] = 1 / L[0][0] * (A[2][0]);
	L[2][1] = 1 / L[1][1] * (A[2][1] - L[2][0] * L[1][0]);
	L[2][2] = sqrt(A[2][2] - (L[2][0] * L[2][0] + L[2][1] * L[2][1]));

	L[0][1] = L[0][2] = L[1][2] = 0;
}

//@tested
//returns the result of the multiplication between A and B in P
void mc_multiplyMatrices(double A[3][3], double B[3][3], double P[3][3]) {
	for (int i = 0; i < 3; ++i) {
		for (int j = 0; j < 3; ++j) {
			P[i][j] = 0;
			for (int k = 0; k < 3; ++k) {
				P[i][j] += A[i][k] * B[k][j];
			}
		}
	}
}

//@tested
//returns the result of the multiplication between A and B in P
void mc_multiplyMatrices(double A[3][3], double B[3], double P[3]) {
	for (int i = 0; i < 3; ++i) {
		P[i] = 0;
		for (int k = 0; k < 3; ++k) {
			P[i] += A[i][k] * B[k];
		}
	}
}

//@tested
//returns the result of the difference between A and B in S
void mc_substractMatrices(double A[3][3], double B[3][3], double S[3][3]) {
	for (int i = 0; i < 3; ++i) {
		for (int j = 0; j < 3; ++j) {
			S[i][j] = A[i][j] - B[i][j];
		}
	}
}

//@tested
//returns the transpose of A in P
void mc_transposeMatrix(double A[3][3], double P[3][3]) {
	for (int i = 0; i < 3; ++i) {
		for (int j = 0; j < 3; ++j) {
			P[i][j] = A[j][i];
		}
	}
}

void mc_substractVector(double A[3], double B[3], double S[3]) {
	for (int i = 0; i < 3; ++i) {
			S[i] = A[i] - B[i];
	}
}

void mc_sumVector(double A[3], double B[3], double S[3]) {
	for (int i = 0; i < 3; ++i) {
			S[i] = A[i] + B[i];
	}
}

void mc_rotation(double point[2],double theta){

	double auxPoint[2];
	auxPoint[0] = point[0];
	auxPoint[1] = point[1];

	double costheta = cos(theta);
	double sintheta = sin(theta);

	point[0] = costheta*auxPoint[0] - sintheta*auxPoint[1];
	point[1] = sintheta*auxPoint[0] + costheta*auxPoint[1];
}

