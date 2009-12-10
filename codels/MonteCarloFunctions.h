#ifndef _MONTE_CARLO_FUNCTIONS_
#define _MONTE_CARLO_FUNCTIONS_

#include "TagPositions.h"

const int mc_nSamples = 200;
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
	mc_Point points[mc_nSamples];
	double weights[mc_nSamples];

	mc_Points() {
		double weight = ((double)1)/((double)mc_nSamples);
		for (int i=0;i<mc_nSamples;++i) {
			weights[i] = weight;
		}
	}
} mc_Points;

mc_Points mc_init(mc_Point origin);

void mc_getEstimatePosition(mc_Points currentPoints,double* mc_x_addr,double* mc_y_addr,double* mc_theta_addr,double mc_cov[3][3]);

bool mc_odometryChangeIsImportant(double dx, double dy, double dtheta, double covariance[3][3]);





#ifdef BOGDANMODEL



mc_Points mc_cycleMainNoMovement(FILE* outputFile,
                                 mc_Points currentPoints,
                                 double sensorModel[NUMBER_OF_ANTENNAS][2*SENSOR_SIZE+1][2*SENSOR_SIZE+1],
                                 TagDetection* tagDetections,
                                 Tag2DVector tags,
                                 TagMap tagmap
                                );

mc_Points mc_cycleMain(FILE* outputFile,
                       mc_Points currentPoints,
                       double dx, double dy, double dtheta, double covariance[3][3],
                       double sensorModel[NUMBER_OF_ANTENNAS][2*SENSOR_SIZE+1][2*SENSOR_SIZE+1],
                       TagDetection* tagDetections,
                       Tag2DVector tags,
                       TagMap tagmap
                       );




#else // #ifdef BOGDANMODEL

mc_Points mc_cycleMainNoMovement(mc_Points currentPoints,
                                 TagDetection* tagDetections,
                                 Tag2DVector tags);

mc_Points mc_cycleMain(mc_Points currentPoints, double odo_Position[3],
		double old_odo[3], double covariance[3][3],
		TagDetection* tagDetections, Tag2DVector tags);





#endif // #ifdef BOGDANMODEL

#endif // _MONTE_CARLO_FUNCTIONS_
