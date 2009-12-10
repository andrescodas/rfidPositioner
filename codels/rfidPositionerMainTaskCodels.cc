/**
 ** rfidPositionerMainTaskCodels.cc
 **
 ** Codels called by execution task rfidPositionerMainTask
 **
 ** Author:
 ** Date:
 **
 **/

#include <portLib.h>
#include <stdio.h>
#include "server/rfidPositionerHeader.h"

#include "server/rfidPositionerHeader.h"
#include "TagPositions.h"
#include "MonteCarloFunctions.h"
#include "input.h"



/*------------------------------------------------------------------------
 *
 * rfidPositionerInit  --  Initialization codel (fIDS, ...)
 *
 * Description:
 *
 * Returns:    OK or ERROR
 */

mc_Points currentPoints;

/*
 *   By: Andres
 *
 *   This variables are related to the Odometry
 *   Maybe Bogdan says "old" because they are the last read
 *
 * */

TagMap tagmap;
Tag2DVector tagvector;

double old_odo[3]; // x y z


/*------------------------
 *  Initializes:
 *  	tagmap  -> Mapping ID an Position of each Tag
 *      tagvector -> Ordered tagmap
 *      Odometry measurements and covariances
 *      Particles Algorithm -> At the point said by the odometry
 *      Sensor Model -> from SENSOR_BASE_MODEL
 *      Writes into OUTPUT_FILE_NAME stuff related to the particle model
 * */
STATUS rfidPositionerInit(int *report) {

	tagmap = initTagMap();
	tagvector = initTag2DVector(tagmap);


	if(initInput() != 0){
		printf("Problems in input initialization\n");
	}

	SDI_F->position.xRob = 0;
	SDI_F->position.yRob = 0;
	SDI_F->position.theta = 0;

	for (int i = 0; i < 5; ++i) {
		SDI_F->estimationError.position_cov[i] = 0;
	}

	currentPoints = mc_init(mc_Point(0, 0, 0));

	initSensorModel();

	return OK;
}

/*------------------------------------------------------------------------
 * ShowTagDetections
 *
 * Description:
 *
 * Reports:      OK
 */

/* rfidPositionerShowTagDetectionsStart  -  codel START of ShowTagDetections
 Returns:  START EXEC END ETHER FAIL ZOMBIE */
ACTIVITY_EVENT rfidPositionerShowTagDetectionsStart(int *report) {
	/* ... add your code here ... */
	return ETHER;
}

/* rfidPositionerShowTagDetectionsMain  -  codel EXEC of ShowTagDetections
 Returns:  EXEC END ETHER FAIL ZOMBIE */
ACTIVITY_EVENT rfidPositionerShowTagDetectionsMain(int *report) {
	/* ... add your code here ... */
	return ETHER;
}

/* rfidPositionerShowTagDetectionsEnd  -  codel END of ShowTagDetections
 Returns:  END ETHER FAIL ZOMBIE */
ACTIVITY_EVENT rfidPositionerShowTagDetectionsEnd(int *report) {
	/* ... add your code here ... */
	return ETHER;
}

/* rfidPositionerShowTagDetectionsInter  -  codel INTER of ShowTagDetections
 Returns:  INTER ETHER FAIL ZOMBIE */
ACTIVITY_EVENT rfidPositionerShowTagDetectionsInter(int *report) {
	/* ... add your code here ... */
	return ETHER;
}

/*------------------------------------------------------------------------
 * TrackPosition
 *
 * Description:
 *
 * Reports:      OK
 */

/* rfidPositionerTrackPositionStart  -  codel START of TrackPosition
 Returns:  START EXEC END ETHER FAIL ZOMBIE */
ACTIVITY_EVENT rfidPositionerTrackPositionStart(int *report) {

	TagDetection* tagDetections = NULL;

	double odo_position[3];
	double odo_cov[3][3];


	if (readRflex(odo_position, odo_cov) != 0){
		return ETHER;
	}
	if(readRFID(&tagDetections)!= 0){
		return ETHER;
	}

	currentPoints = mc_cycleMain(currentPoints, odo_position, old_odo, odo_cov,tagDetections, tagvector);


	double mc_x, mc_y, mc_theta, mc_cov[3][3];

	mc_getEstimatePosition(currentPoints, &mc_x, &mc_y, &mc_theta, mc_cov);

	//	printf("[%05d](%+010.5lf,%+010.5lf,%+010.5lf)(%+010.5lf,%+010.5lf,%+010.5lf)\n",cycleCounter,mc_x,mc_y,mc_theta,odo_position_x,odo_position_y,odo_position_theta);

	//mc_printSamples(NULL, currentPoints);
	//	fprintf(outputFile, "Odometer position: (%lf, %lf, %lf)\n",odo_position_x, odo_position_y, odo_position_theta);
	//	fprintf(outputFile, "Estimated position: (%lf, %lf, %lf)\n",mc_x, mc_y, mc_theta);

	SDI_F->position.xRob = mc_x;
	SDI_F->position.yRob = mc_y;
	SDI_F->position.theta = mc_theta;

	/* Set new variances on (x,y,theta)
	 With : v0 = vxx;  v1 = vxy;  v2 = vyy;  v3 = vxt; v4 = vyt;  v5 = vtt */
	SDI_F->estimationError.position_cov[0] = mc_cov[0][0];
	SDI_F->estimationError.position_cov[1] = mc_cov[0][1];
	SDI_F->estimationError.position_cov[2] = mc_cov[1][1];
	SDI_F->estimationError.position_cov[3] = mc_cov[0][2];
	SDI_F->estimationError.position_cov[4] = mc_cov[1][2];
	SDI_F->estimationError.position_cov[5] = mc_cov[2][2];

	old_odo[0] = odo_position[0];
	old_odo[1] = odo_position[1];
	old_odo[2] = odo_position[2];

	freeTagDetection(tagDetections);

	return ETHER;
}

/* rfidPositionerTrackPositionMain  -  codel EXEC of TrackPosition
 Returns:  EXEC END ETHER FAIL ZOMBIE */
ACTIVITY_EVENT rfidPositionerTrackPositionMain(int *report) {
	/* ... add your code here ... */
	return ETHER;
}

/* rfidPositionerTrackPositionEnd  -  codel END of TrackPosition
 Returns:  END ETHER FAIL ZOMBIE */
ACTIVITY_EVENT rfidPositionerTrackPositionEnd(int *report) {
	/* ... add your code here ... */
	return ETHER;
}

/* rfidPositionerTrackPositionInter  -  codel INTER of TrackPosition
 Returns:  INTER ETHER FAIL ZOMBIE */
ACTIVITY_EVENT rfidPositionerTrackPositionInter(int *report) {
	/* ... add your code here ... */
	return ETHER;
}

/*------------------------------------------------------------------------
 * StartParticules
 *
 * Description:
 *
 * Reports:      OK
 */

/* rfidStartParticulesStart  -  codel START of StartParticules
 Returns:  START EXEC END ETHER FAIL ZOMBIE */
ACTIVITY_EVENT rfidStartParticulesStart(POSITION *position, int *report) {

	currentPoints = mc_init(mc_Point(position->xRob, position->yRob,
			position->theta));

	old_odo[0] = position->xRob;
	old_odo[1] = position->yRob;
	old_odo[2] = position->theta;

	SDI_F->position.xRob = position->xRob;
	SDI_F->position.yRob = position->yRob;
	SDI_F->position.theta = position->theta;

	/* Set new variances on (x,y,theta)
	 With : v0 = vxx;  v1 = vxy;  v2 = vyy;  v3 = vxt; v4 = vyt;  v5 = vtt */
	SDI_F->estimationError.position_cov[0] = 0;
	SDI_F->estimationError.position_cov[1] = 0;
	SDI_F->estimationError.position_cov[2] = 0;
	SDI_F->estimationError.position_cov[3] = 0;
	SDI_F->estimationError.position_cov[4] = 0;
	SDI_F->estimationError.position_cov[5] = 0;

	return ETHER;
}

