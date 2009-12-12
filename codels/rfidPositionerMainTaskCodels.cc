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

#include "TagPositions.h"
#include "MonteCarloFunctions.h"
#include "input.h"
#include <stdio.h>
#include "server/rfidPositionerHeader.h"


mc_Points currentPoints;
TagMap tagmap;
Tag2DVector tagvector;

double old_odo[3]; // x y t

/*------------------------------------------------------------------------
 *
 * rfidPositionerInit  --  Initialization codel (fIDS, ...)
 *
 * Description:
 *
 * Returns:    OK or ERROR
 */


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

	if (initInput() != 0) {
		printf("Problems in input initialization\n");
	}

	SDI_F->position.xRob = 0;
	SDI_F->position.yRob = 0;
	SDI_F->position.theta = 0;

	for (int i = 0; i < 5; ++i) {
		SDI_F->estimationError.position_cov[i] = 0;
	}

	mc_init(mc_Point(0, 0, 0),&currentPoints);

	initSensorModel();

	return OK;
}

/*------------------------------------------------------------------------
 * TrackPosition
 *
 * Description:
 *
 * Reports:      OK
 *              S_rfidPositioner_RFID_POSTER_NOT_FOUND
 *              S_rfidPositioner_RFLEX_POSTER_NOT_FOUND
 */

/* rfidPositionerTrackPositionStart  -  codel START of TrackPosition
 Returns:  START EXEC END ETHER FAIL ZOMBIE */
ACTIVITY_EVENT rfidPositionerTrackPositionStart(int *report) {

	TagDetection* tagDetections = NULL;
	double odo_position[3];
	double odo_cov[3][3];


	if (readRflex(odo_position, odo_cov) != 0) {
		return ETHER;
	}
	if (readRFID(&tagDetections) != 0) {
		return ETHER;
	}

	mc_cycleMain(&currentPoints, odo_position, old_odo, odo_cov,
			tagDetections, tagvector);


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

/* rfidPositionerTrackPositionInter  -  codel INTER of TrackPosition
   Returns:  INTER ETHER FAIL ZOMBIE */
ACTIVITY_EVENT
rfidPositionerTrackPositionInter(int *report)
{
  printf("rfidPositionerTrackPositionInter\n");
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
ACTIVITY_EVENT
rfidStartParticulesStart(POSITION *position, int *report)
{
	mc_init(mc_Point(position->xRob, position->yRob,
			position->theta),&currentPoints);

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

/*------------------------------------------------------------------------
 * SetNumberParticles
 *
 * Description:
 *
 * Reports:      OK
 */

/* rfidPositionerSetNumberParticlesStart  -  codel START of SetNumberParticles
   Returns:  START EXEC END ETHER FAIL ZOMBIE */
ACTIVITY_EVENT
rfidPositionerSetNumberParticlesStart(int *numberParticles, int *report)
{
  mc_nSamples = *numberParticles;
  return ETHER;
}

/* rfidPositionerSetNumberParticlesInter  -  codel INTER of SetNumberParticles
   Returns:  INTER ETHER FAIL ZOMBIE */
ACTIVITY_EVENT
rfidPositionerSetNumberParticlesInter(int *numberParticles, int *report)
{
  printf("rfidPositionerSetNumberParticlesInter\n");
  return ETHER;
}

/*------------------------------------------------------------------------
 * GetNumberParticles
 *
 * Description:
 *
 * Reports:      OK
 */

/* rfidPositionerGetNumberParticlesStart  -  codel START of GetNumberParticles
   Returns:  START EXEC END ETHER FAIL ZOMBIE */
ACTIVITY_EVENT
rfidPositionerGetNumberParticlesStart(int *numberParticles, int *report)
{
  *numberParticles = mc_nSamples;
  return ETHER;
}

/* rfidPositionerGetNumberParticlesInter  -  codel INTER of GetNumberParticles
   Returns:  INTER ETHER FAIL ZOMBIE */
ACTIVITY_EVENT
rfidPositionerGetNumberParticlesInter(int *numberParticles, int *report)
{
  printf("rfidPositionerGetNumberParticlesInter\n");
  return ETHER;
}


