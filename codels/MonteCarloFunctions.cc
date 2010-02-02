#include "MonteCarloFunctions.h"
#include "MonteCarloPrintFunctions.h"
#include "MonteCarloMath.h"

#include <stdio.h>
#include <stdlib.h>
#include <time.h>
#include <string.h>

int mc_nSamples = 200;

/*
// @ Revised by andres
//draws mc_nSamples points from the current samples uniformely according to their weights
mc_Points mc_generateNextUniformExtractedPoints(mc_Points p) {
	mc_Points result;
	double f;
	double s;
	double weight = ((double) 1) / (double) mc_nSamples;
	mc_Point crtPoint;
	int k;

	for (int i = 0; i < mc_nSamples; ++i) {
		f = mc_getRandomUniformDouble();
		s = 0;
		k = -1;

		do {
			++k;
			s += p.weights[k];
		} while (s < f);
		crtPoint = p.points[k];

		result.points[i] = crtPoint;
		result.weights[i] = weight;
	}

	return result;
}
*/

//returns a copy of the points source
void mc_copyPoints(const mc_Points *source, mc_Points *result) {
	for (int i = 0; i < mc_nSamples; ++i) {
		result->points[i] = source->points[i];
		result->weights[i] = source->weights[i];
	}
}

void mc_cumulativeWeights(mc_Points *p) {
	double sum = 0;
	for (int i = 0; i < mc_nSamples; i++) {
		sum = sum + p->weights[i];
		p->weights[i] = sum;
	}

	if (fabs(sum - 1) > 0.000001) {
		printf("Warning sum of sums == %.20lf\n", sum);
	}

}

int searchParticule(const mc_Points *p, double aleatoryNumber) {
	int bottomIndex = 0;
	int topIndex = mc_nSamples - 1;
	int currentIndex;

	if (aleatoryNumber < p->weights[0]) {
		//printf("\nFirst index Weight == %.30lf\n",p->weights[0]);
		//printf("Warning aleatory number    == %.30lf\n",aleatoryNumber);
		return 0;
	}

	if (aleatoryNumber > p->weights[topIndex]) {
		printf("\nWarning top index Weight == %.30lf\n", p->weights[topIndex]);
		printf("Warning aleatory number    == %.30lf\n", aleatoryNumber);
		return topIndex;
	}

	while (topIndex - bottomIndex > 1) {
		currentIndex = (int) floor(double((topIndex + bottomIndex) / 2));

		if (p->weights[currentIndex] < aleatoryNumber) {
			bottomIndex = currentIndex;
		} else {
			topIndex = currentIndex;
		}
	}

	return topIndex;
}


void mc_generateNextUniformExtractedPoints(mc_Points *p) {

	int pIndex = 0;
	double aleatoryNumber;
	mc_Points results;

	double weight = (double) 1 / (double) mc_nSamples;
	mc_cumulativeWeights(p);

	for (int k = 0; k < mc_nSamples; k++) {
		aleatoryNumber = mc_getRandomUniformDouble();
		pIndex = searchParticule(p, aleatoryNumber);

		results.points[k].x = p->points[pIndex].x;
		results.points[k].y = p->points[pIndex].y;
		results.points[k].theta = p->points[pIndex].theta;
		results.weights[k] = weight;
	}

	mc_copyPoints(&results,p);
}

//@tested
//normalizes the data (normalizes the weigths and wraps the angles)
void mc_normalize(mc_Points *p) {
	double sum = 0;
	double weight;

	for (int i = 0; i < mc_nSamples; ++i) {
		sum += p->weights[i];
	}

	if (sum > 0) {
		for (int i = 0; i < mc_nSamples; ++i) {
			p->points[i].theta = angleWrap(p->points[i].theta);
			p->weights[i] /= sum;
		}
	} else {
		printf("All particles weight sum up to 0\n");
		printf("Reinitializing particles weight with equal weights\n");
		weight = (double) 1 / (double) mc_nSamples;
		for (int i = 0; i < mc_nSamples; ++i) {
			p->points[i].theta = angleWrap(p->points[i].theta);
			p->weights[i] = weight;
		}
	}
}

//@tested
//initializes the points (currently the initialization corresponds to the case where we now the starting position: origin)
void mc_init(const mc_Point origin,mc_Points *particles) {
	srand(time(NULL));
	double weight = ((double) 1) / (double) mc_nSamples;
	for (int i = 0; i < mc_nSamples; ++i) {
		particles->points[i] = origin;
		particles->weights[i] = weight;
	}
}

//executes the odometry related step of MonteCarlo: draws for each point a point in its odometry area of incertitude
// <- currentPoints = the current points
// <- dx,dy,dtheta = the measured odometrical movements
// <- covariance = the error estimation of the odometer
// -> the generated points
void mc_generateNextOdometryPoints(mc_Points *currentPoints,
		const double odo_Position[3], const double old_odo[3], const double covariance[3][3]) {

	double deslocInRflexBase[3];
	double distributionPoint[3];
	double auxPoint[3];

	mc_substractVector(odo_Position, old_odo, deslocInRflexBase);

	for (int i = 0; i < mc_nSamples; ++i) {

		mc_generateGaussianRandomPoint(distributionPoint, covariance);

		mc_sumVector(deslocInRflexBase, distributionPoint, auxPoint);

		mc_rotation(auxPoint, currentPoints->points[i].theta - old_odo[2]);

		currentPoints->points[i] = mc_Point(currentPoints->points[i].x + auxPoint[0],
				currentPoints->points[i].y + auxPoint[1],
				currentPoints->points[i].theta + auxPoint[2]);
	}
}

//executes the observation related step of MonteCarlo: adjusts the weight of each sample according to the similarity it holds to the actual observations
// <- currentPoints = the current points
// <- sensorModel
// <- tagDetections = the detections received in reality by the antennas
// <- number_of_detected_tags = the length of the former array
// <- tags
// <- tagmap


//calculates the position given by the samples and their weights and returns it in the pointer parameters
void mc_getEstimatePosition(const mc_Points currentPoints, double* mc_x_addr,
		double* mc_y_addr, double* mc_theta_addr, double mc_cov[3][3]) {
	*mc_x_addr = 0;
	*mc_y_addr = 0;
	*mc_theta_addr = 0;

	double theta_cos = 0;
	double theta_sin = 0;

	double exy = 0;
	double exx = 0;
	double eyy = 0;

	double ext = 0;
	double eyt = 0;
	double ett = 0;
	double desvTheta_aux = 0;

	for (int i = 0; i < mc_nSamples; ++i) {
		(*mc_x_addr) += currentPoints.points[i].x * currentPoints.weights[i];
		(*mc_y_addr) += currentPoints.points[i].y * currentPoints.weights[i];
		exy += currentPoints.points[i].x * currentPoints.points[i].y
				* currentPoints.weights[i];
		exx += currentPoints.points[i].x * currentPoints.points[i].x
				* currentPoints.weights[i];
		eyy += currentPoints.points[i].y * currentPoints.points[i].y
				* currentPoints.weights[i];
		theta_cos += cos(currentPoints.points[i].theta)
				* currentPoints.weights[i];
		theta_sin += sin(currentPoints.points[i].theta)
				* currentPoints.weights[i];
	}

	(*mc_theta_addr) = atan2(theta_sin, theta_cos);

	mc_cov[0][0] = exx - (*mc_x_addr) * (*mc_x_addr);
	mc_cov[1][1] = eyy - (*mc_y_addr) * (*mc_y_addr);
	mc_cov[0][1] = mc_cov[1][0] = exy - (*mc_x_addr) * (*mc_y_addr);

	for (int i = 0; i < mc_nSamples; ++i) {
		desvTheta_aux = angleWrap(currentPoints.points[i].theta
				- *mc_theta_addr);

		ext += currentPoints.points[i].x * desvTheta_aux
				* currentPoints.weights[i];
		eyt += currentPoints.points[i].y * desvTheta_aux
				* currentPoints.weights[i];
		ett += desvTheta_aux * desvTheta_aux * currentPoints.weights[i];

	}

	mc_cov[1][2] = mc_cov[2][1] = eyt - (*mc_y_addr) * (*mc_theta_addr);
	mc_cov[0][2] = mc_cov[2][0] = ext - (*mc_x_addr) * (*mc_theta_addr);
	mc_cov[2][2] = ett;
}



// TODO : Is really important ?  --> considering always true  --> odometry study will answer
/*the function returns true if the change reported by the odometer should be immediately reflected
 * in monte carlo (the odometry step should be executed) or we should wait until the change
 * is more important
 * */
bool mc_odometryChangeIsImportant(double dx, double dy, double dtheta,
		const double covariance[3][3]) {
	if (1 == 1) {
		return true;
	}
	double L[3][3];
	mc_getCholeskyDecomposition(covariance, L);
	if ((fabs(dx) < L[0][0] / 10) && (fabs(dy) < L[1][1] / 10) && (fabs(dtheta)
			< L[2][2] / 10)) {
		return false;
	} else {
		return true;
	}
}

double mc_getSimilarityProbability(const TagDetection* tagDetections,
		TagExpectation* tagsExpected, int created) {

	double p = 1;
	int deleted = 0;
	int founded = 0;
	TagExpectation* tagExpectationAux = tagsExpected;
	TagExpectation* notFoundedYet = tagsExpected;
	TagExpectation* before = NULL;
	const TagDetection* tagDetectionAux = tagDetections;

	// TODO: CHANGE WHEN ANTENNA 5 IS REPAIRED
	// for all the possible detections
	while (tagDetectionAux != NULL) {

		// remove when antenna 5 repaired begin
		while (tagDetectionAux->antenna == 5 || tagDetectionAux->antenna == 5
				|| tagDetectionAux->antenna == 5) {
			tagDetectionAux = tagDetectionAux->next;
			if (tagDetectionAux == NULL) {
				break;
			}
		}
		if (tagDetectionAux == NULL) {
			break;
		}
		// remove when antenna 5 repaired end
		tagExpectationAux = notFoundedYet;
		before = NULL;
		founded = 0;
		// for all the detected tags that weren't find yet in the expected list
		while (tagExpectationAux != NULL) {

			// search if pair correspond a pair possible detection -- detection
			if ((strcmp(tagExpectationAux->tagid, tagDetectionAux->tagid) == 0)
					&& (tagExpectationAux->antenna == tagDetectionAux->antenna)) {

				founded = 1;

				if (before == NULL) {
					before = notFoundedYet;
					notFoundedYet = notFoundedYet->next;
					free(before);
					deleted = deleted + 1;
					before = NULL;
				} else {
					before->next = tagExpectationAux->next;
					free(tagExpectationAux);
					deleted = deleted + 1;
				}
				break;
			} else {
				before = tagExpectationAux;
			}

			tagExpectationAux = tagExpectationAux->next;
		}
		if (founded) {
			p *= tagExpectationAux->probability; // tag founded --> return likelihood of founding it;
		} else {
			p *= mc_PENALIZING_FACTOR;
		}
		tagDetectionAux = tagDetectionAux->next;
	}

	tagExpectationAux = notFoundedYet;

	// penalize if found a non expecting tag
	while (tagExpectationAux != 0) {

		/// TODO: remove when antenna 5 repaired begin
		while (tagExpectationAux->antenna == 5 || tagExpectationAux->antenna
				== 5 || tagExpectationAux->antenna == 5) {
			before = tagExpectationAux;
			tagExpectationAux = tagExpectationAux->next;
			free(before);
			deleted = deleted + 1;
			if (tagExpectationAux == NULL) {
				break;
			}
		}
		if (tagExpectationAux == NULL) {
			break;
		}
		/// remove when antenna 5 repaired  end

		p *= (1 - tagExpectationAux->probability);

		before = tagExpectationAux;
		tagExpectationAux = tagExpectationAux->next;
		free(before);
		deleted = deleted + 1;
	}
	if (created != deleted) {
		printf("created = %d != deleted = %d", created, deleted);
	}

	return p;
}
// created int variable is only for verification... debug
void mc_adjustWeightsThroughObservation(mc_Points *currentPoints,
		const TagDetection* tagDetections,const  Tag2DVector tags) {

	TagExpectation* tagsExpected;
	int created;

	double p;

	for (int i = 0; i < mc_nSamples; ++i) {

		tagsExpected = NULL;

		created = getExpectedTagDetections(&tagsExpected, currentPoints->points[i].x,
				currentPoints->points[i].y, currentPoints->points[i].theta, tags);

		p = mc_getSimilarityProbability(tagDetections, tagsExpected, created);

		currentPoints->weights[i] = currentPoints->weights[i] * p;
	}
}

int stepIndex = 0;

//the main Monte Carlo cycle without the odometry step
// <- outputFile = file for debugging output
// <- currentPoints
// <- sensorModel
// <- tagDetections = the detections received in reality by the antennas
// <- number_of_detected_tags = the length of the former array
// <- tags
// <- tagmap
// -> the new points after a Monte Carlo cycle

void mc_cycleMainNoMovement(mc_Points *currentPoints,
		const TagDetection* tagDetections,const  Tag2DVector tags) {

	char outputFile[64];

	//mc_printSamples("Before observation:",result);

	mc_adjustWeightsThroughObservation(currentPoints, tagDetections, tags);
	//mc_printSamples("After observation:",result);

	sprintf(outputFile, "afterWeight%d.m", stepIndex);
	//mc_printSamples(outputFile, currentPoints);

	mc_normalize(currentPoints);

	sprintf(outputFile, "afterNormalize%d.m", stepIndex);
	//mc_printSamples(outputFile, currentPoints);



	mc_generateNextUniformExtractedPoints(currentPoints);



	sprintf(outputFile, "afterResample%d.m", stepIndex);
	//mc_printSamples(outputFile, currentPoints);

}



//the main Monte Carlo cycle
// <- outputFile = file for debugging output
// <- currentPoints
// <- dx, dy, dtheta, double covariance[3][3] = odometry data (movement and error)
// <- sensorModel
// <- tagDetections = the detections received in reality by the antennas
// <- number_of_detected_tags = the length of the former array
// <- tags
// <- tagmap
// -> the new points after a Monte Carlo cycle


void mc_cycleMain(mc_Points *currentPoints,const double odo_Position[3],
		const double old_odo[3],const  double covariance[3][3],
		const TagDetection* tagDetections,const  Tag2DVector tags) {

	int sprintfResult;
	char outputFile[64];

	stepIndex++;

	sprintfResult = sprintf(outputFile, "initStep%d.m",stepIndex);
	//mc_printSamples(outputFile, currentPoints);


	mc_generateNextOdometryPoints(currentPoints,odo_Position, old_odo, covariance);


	sprintf(outputFile, "afterOdometry%d.m", stepIndex);
	//mc_printSamples(outputFile, currentPoints);


	mc_cycleMainNoMovement(currentPoints, tagDetections, tags);

}
