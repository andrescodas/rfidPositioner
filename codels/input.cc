/*
 * input.cc
 *
 *  Created on: 30 nov. 2009
 *      Author: andres
 */

#include "input.h"

//#define real
#ifdef real

POSTER_ID rflexPosterId;
POSTER_ID rfidPosterId;

int initInput() {

	int report;
	char error[256];

	char rfidPosterName[256];
	sprintf(rfidPosterName, "rfidTagList");
	if (posterFind(rfidPosterName, (POSTER_ID *) &rfidPosterId) == ERROR) // If rfidPosterName is not found
	{
		sprintf(error,
				"[rfidPositionerPosterUpdateTaskInit] posterFind(rfid) failed");
		h2perror(error);
		report = errnoGet();
		return 1;
	}

	char rflexPosterName[256];
	sprintf(rflexPosterName, "rflexRobot");
	if (posterFind(rflexPosterName, (POSTER_ID *) &rflexPosterId) == ERROR) {
		sprintf(error,
				"[rfidPositionerPosterUpdateTaskInit] posterFind(rflex) failed");
		h2perror(error);
		report = errnoGet();
		return 1;
	}

	RFID_TAGLIST_POSTER_STR* rfidPoster =
	(RFID_TAGLIST_POSTER_STR*) posterAddr(rfidPosterId); // Get rfidPosterId Address
	if (rfidPoster == NULL) {
		sprintf(error,
				"[rfidPositionerPosterUpdateTaskInit] posterAddr(rfid) failed)");
		h2perror(error);
		return 1;
	}

	RFLEX_ROBOT_POSTER_STR* rflexPoster = (RFLEX_ROBOT_POSTER_STR*) posterAddr(
			rflexPosterId);
	if (rflexPoster == NULL) {
		sprintf(error,
				"[rfidPositionerPosterUpdateTaskInit] posterAddr(rflex) failed");
		h2perror(error);

		return 1;
	}
}

int readRflex(double odo_position[3], double odo_cov[3][3]) {
	char error[256];
	int report;
	RFLEX_ROBOT_POSTER_STR* rflexPoster = (RFLEX_ROBOT_POSTER_STR*) posterAddr(
			rflexPosterId);

	if (rflexPoster == NULL) {
		sprintf(error,
				"[rfidPositionerPosterUpdateTaskMain] posterAddr(rflex) failed");
		h2perror(error);
		report = S_rfidPositioner_RFLEX_POSTER_NOT_FOUND;
		return ETHER;
	}

	posterTake(rflexPosterId, POSTER_READ);
	odo_cov[0][0] = rflexPoster->PosError.var[0]; //xx
	odo_cov[1][1] = rflexPoster->PosError.var[2]; //yy
	odo_cov[2][2] = rflexPoster->PosError.var[5]; //tt
	odo_cov[0][1] = odo_cov[1][0] = rflexPoster->PosError.var[1]; //xy
	odo_cov[0][2] = odo_cov[2][0] = rflexPoster->PosError.var[3]; //xt
	odo_cov[1][2] = odo_cov[2][1] = rflexPoster->PosError.var[4]; //yt

	odo_position[0] = rflexPoster->Position.xRob;
	odo_position[1] = rflexPoster->Position.yRob;
	odo_position[2] = rflexPoster->Position.theta;
	posterGive(rflexPosterId);
}

int readRFID(TagDetection** tagDetections) {
	char error[256];
	int report;
	RFID_TAGLIST_POSTER_STR* rfidPoster =
	(RFID_TAGLIST_POSTER_STR*) posterAddr(rfidPosterId);

	if (rfidPoster == NULL) {
		sprintf(error,
				"[rfidPositionerPosterUpdateTaskMain] posterAddr(rfid) failed");
		h2perror(error);
		report = S_rfidPositioner_RFID_POSTER_NOT_FOUND;
		return 1;
	}

	posterTake(rfidPosterId, POSTER_READ);
	getTagDetections(tagDetections, rfidPoster);
	posterGive(rfidPosterId);

}

#else
#include <stdio.h>
#include <stdlib.h>

FILE* covFile;
FILE* odoFile;
FILE* rfidFile;

int initInput() {

	covFile = fopen("~/simulation/rflexCovarianceSet.txt", "r");
	odoFile = fopen("~/simulation/rflexPositionSet.txt", "r");
	rfidFile = fopen("~/simulation/detectionsBogdanTest.txt", "r");

	if (covFile == NULL) {
		fprintf(stderr,"ERROR: Could not open the file ~/simulation/rflexCovarianceSet.txt \n");
		return 1;
	}
	if (odoFile == NULL) {
		fprintf(stderr,"ERROR: Could not open the file ~/simulation/rflexPositionSet.txt \n");
		return 1;
	}
	if (rfidFile == NULL) {
		fprintf(
				stderr,
				"ERROR: Could not open the file ~/simulation/detectionsBogdanTest.txt \n");
		return 1;
	}

	return 0;

}
int readRflex(double odo_position[3], double odo_cov[3][3]) {
	int index;
	int status;

	status = fscanf(odoFile, "%u", &index);
	if (status != 1) {
		printf("error reading inputfile\n");
		return -1;
	}

	status = fscanf(odoFile, "%lf", &odo_position[0]);
	if (status != 1) {
		printf("error reading inputfile\n");
		return -1;
	}

	status = fscanf(odoFile, "%lf", &odo_position[1]);
	if (status != 1) {
		printf("error reading inputfile\n");
		return -1;
	}
	status = fscanf(odoFile, "%lf", &odo_position[2]);

	if (status != 1) {
		printf("error reading inputfile\n");
		return -1;
	}

	status = fscanf(covFile, "%u", &index);
	if (status != 1) {
		printf("error reading inputfile\n");
		return -1;
	}

	status = fscanf(covFile, "%lf", &odo_cov[0][0]);
	if (status != 1) {
		printf("error reading inputfile\n");
		return -1;
	}
	status = fscanf(covFile, "%lf", &odo_cov[1][0]);
	if (status != 1) {
		printf("error reading inputfile\n");
		return -1;
	}
	odo_cov[0][1] = odo_cov[1][0];

	status = fscanf(covFile, "%lf", &odo_cov[1][1]);
	if (status != 1) {
		printf("error reading inputfile\n");
		return -1;
	}
	status = fscanf(covFile, "%lf", &odo_cov[2][0]);
	if (status != 1) {
		printf("error reading inputfile\n");
		return -1;
	}
	odo_cov[0][2] = odo_cov[2][0];

	status = fscanf(covFile, "%lf", &odo_cov[2][1]);
	if (status != 1) {
		printf("error reading inputfile\n");
		return -1;
	}
	odo_cov[1][2] = odo_cov[2][1];

	status = fscanf(covFile, "%lf", &odo_cov[2][2]);
	if (status != 1) {
		printf("error reading inputfile\n");
		return -1;
	}
	return 0;
}

int readRFID(TagDetection** tagDetections) {

	TagDetection* before = NULL;
	TagDetection* detected = NULL;
	int detection;
	char tagid[TAG_ID_MAX_SIZE + 1];

	int status;

	fpos_t rfidInitial;
	int rfidInitialStatus;

	double posxB;
	double posyB;
	double postB;
	double posx = -999;
	double posy = -999;
	double post = -999;

	status = fscanf(rfidFile, "%lf %lf %lf", &posx, &posy, &post);

	if (status != 3) {
		printf("Problem Reading RFID File! PositionsInit, number read = %d\n",
				status);
		printf("Given Positions = %lf %lf %lf\n", posx, posy, post);
		return 1;
	}
	posxB = posx;
	posyB = posy;
	postB = post;

	while ((posxB == posx) && (posyB == posy) && (postB == post)) {

		status = fscanf(rfidFile, "%s", tagid);
		if (status != 1) {
			printf("Problem Reading RFID File! TagId, number read = %d\n",
					status);
			printf("TagRead = %s", tagid);
			return 1;
		}

		for (int j = 0; j < NUMBER_OF_ANTENNAS; ++j) {
			status = fscanf(rfidFile, "%u", &detection);
			if (status != 1) {
				printf(
						"Problem Reading RFID File! Detection, number read = %d\n",
						status);
				printf("Detection = %d", detection);
				return 1;
			}
			if (detection == 1) {
				if (*tagDetections == NULL) { // there is not before

					detected = (TagDetection*) malloc(sizeof(TagDetection));
					*tagDetections = detected;
				} else {
					before = detected;
					detected = (TagDetection*) malloc(sizeof(TagDetection));
					before->next = detected;
				}

				*detected = TagDetection(j, tagid, NULL);
			}
		}

		posxB = posx;
		posyB = posy;
		postB = post;

		rfidInitialStatus = fgetpos(rfidFile, &rfidInitial);

		if (rfidInitialStatus != 0) {
			printf("Problems getting file Position = %d\n", rfidInitialStatus);
			return 1;
		}

		status = fscanf(rfidFile, "%lf %lf %lf", &posx, &posy, &post);
		if (status != 3) {
			printf(
					"Problem Reading RFID File! PositionsInit, number read = %d\n",
					status);
			printf("Given Positions = %lf %lf %lf\n", posx, posy, post);
			return 1;
		}

	}
	if (rfidInitialStatus != 0) {
		printf("Problem Reading RFID File! End\n");
		return -1;
	} else {
		return fsetpos(rfidFile, &rfidInitial);
	}
	return 0;
}

#endif
