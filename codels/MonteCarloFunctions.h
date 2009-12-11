#ifndef _MONTE_CARLO_FUNCTIONS_
#define _MONTE_CARLO_FUNCTIONS_

#include "TagPositions.h"

const int MAXMCNSAMPLES = 4000;
extern int mc_nSamples;

const double mc_PENALIZING_FACTOR = 0;//0.001;

typedef struct mc_Point {
	double x,y,theta;

	mc_Point(double _x, double _y, double _theta) {
		x=_x;
		y=_y;
		theta=_theta;
	}
	mc_Point() {
		x=0;
		y=0;
		theta=0;
	}
} mc_Point;

typedef struct mc_Points {
	mc_Point points[MAXMCNSAMPLES];
	double weights[MAXMCNSAMPLES];

	mc_Points() {

	}
} mc_Points;

void mc_init(mc_Point origin,mc_Points *particles);

void mc_getEstimatePosition(mc_Points currentPoints,double* mc_x_addr,double* mc_y_addr,double* mc_theta_addr,double mc_cov[3][3]);

bool mc_odometryChangeIsImportant(double dx, double dy, double dtheta, double covariance[3][3]);

void mc_cycleMain(mc_Points *currentPoints, double odo_Position[3],
		double old_odo[3], double covariance[3][3],
		TagDetection* tagDetections, Tag2DVector tags);

#endif // _MONTE_CARLO_FUNCTIONS_
