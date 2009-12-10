#include "MonteCarloFunctions.h"
#include "MonteCarloPrintFunctions.h"
#include "MonteCarloMath.h"

#include <stdio.h>
#include <stdlib.h>
#include <time.h>
#include <string.h>

// @ Revised by andres
//draws mc_nSamples points from the current samples uniformely according to their weights
mc_Points mc_generateNextUniformExtractedPoints(mc_Points p) {
	mc_Points result;
	double f;
	double s;
	mc_Point crtPoint;
	int k;

	for (int i = 0; i < mc_nSamples; ++i) {
		f = mc_getRandomUniformDouble();
		s = 0;
		k = 0;

		do {
			crtPoint = p.points[k];
			s += p.weights[k];
			++k;
		} while (s < f);

		result.points[i] = crtPoint;
		result.weights[i] = ((double) 1) / (double) mc_nSamples;
	}

	return result;
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

int searchParticule(mc_Points *p, double aleatoryNumber) {
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

mc_Points mc_generateNextUniformExtractedPoints2(mc_Points *p) {

	int pIndex = 0;
	double aleatoryNumber;
	mc_Points result;

	mc_cumulativeWeights(p);

	for (int k = 0; k < mc_nSamples; k++) {
		aleatoryNumber = mc_getRandomUniformDouble();
		pIndex = searchParticule(p, aleatoryNumber);

		result.points[k].x = p->points[pIndex].x;
		result.points[k].y = p->points[pIndex].y;
		result.points[k].theta = p->points[pIndex].theta;
	}

	return result;
}

//@tested
//normalizes the data (normalizes the weigths and wraps the angles)
void mc_normalize(mc_Points p[]) {
	double sum = 0;
	double weight;

	for (int i = 0; i < mc_nSamples; ++i) {
		sum += p->weights[i];
	}

	printf("sum=%.30lf\n", sum);

	if (sum > 0) {
		for (int i = 0; i < mc_nSamples; ++i) {
			p->points[i].theta = angleWrap(p->points[i].theta);
			p->weights[i] /= sum;
		}
	} else {
		weight = (double) 1 / (double) mc_nSamples;
		for (int i = 0; i < mc_nSamples; ++i) {
			p->points[i].theta = angleWrap(p->points[i].theta);
			p->weights[i] = weight;
		}
	}
}

//@tested
//initializes the points (currently the initialization corresponds to the case where we now the starting position: origin)
mc_Points mc_init(mc_Point origin) {
	srand(time(NULL));
	mc_Points result;
	for (int i = 0; i < mc_nSamples; ++i) {
		result.points[i] = origin;
		result.weights[i] = ((double) 1) / (double) mc_nSamples;
	}
	return result;
}

//executes the odometry related step of MonteCarlo: draws for each point a point in its odometry area of incertitude
// <- currentPoints = the current points
// <- dx,dy,dtheta = the measured odometrical movements
// <- covariance = the error estimation of the odometer
// -> the generated points
mc_Points mc_generateNextOdometryPoints(mc_Points currentPoints,
		double odo_Position[3], double old_odo[3], double covariance[3][3]) {
	mc_Points result;
	double deslocInRflexBase[3];
	double distributionPoint[3];
	double auxPoint[3];

	mc_substractVector(odo_Position, old_odo, deslocInRflexBase);

	for (int i = 0; i < mc_nSamples; ++i) {

		mc_generateGaussianRandomPoint(distributionPoint, covariance);

		mc_sumVector(deslocInRflexBase, distributionPoint, auxPoint);

		mc_rotation(auxPoint, currentPoints.points[i].theta - old_odo[2]);

		result.points[i] = mc_Point(currentPoints.points[i].x + auxPoint[0],
				currentPoints.points[i].y + auxPoint[1],
				currentPoints.points[i].theta + auxPoint[2]);

		result.weights[i] = currentPoints.weights[i];
	}
	return result;
}

//executes the observation related step of MonteCarlo: adjusts the weight of each sample according to the similarity it holds to the actual observations
// <- currentPoints = the current points
// <- sensorModel
// <- tagDetections = the detections received in reality by the antennas
// <- number_of_detected_tags = the length of the former array
// <- tags
// <- tagmap


//calculates the position given by the samples and their weights and returns it in the pointer parameters
void mc_getEstimatePosition(mc_Points currentPoints, double* mc_x_addr,
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

//returns a copy of the points source
mc_Points mc_copyPoints(mc_Points source) {
	mc_Points result;
	for (int i = 0; i < mc_nSamples; ++i) {
		result.points[i] = source.points[i];
		result.weights[i] = source.weights[i];
	}
	return result;
}

// TODO : Is really important ?  --> considering always true  --> odometry study will answer
/*the function returns true if the change reported by the odometer should be immediately reflected
 * in monte carlo (the odometry step should be executed) or we should wait until the change
 * is more important
 * */
bool mc_odometryChangeIsImportant(double dx, double dy, double dtheta,
		double covariance[3][3]) {
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

double mc_getSimilarityProbability(TagDetection* tagDetections,
		TagExpectation* tagsExpected, mc_Point robot_position, int created) {

	double p = 1;
	int deleted = 0;
	int founded = 0;
	TagExpectation* tagExpectationAux = tagsExpected;
	TagExpectation* notFoundedYet = tagsExpected;
	TagExpectation* before = NULL;
	TagDetection* tagDetectionAux = tagDetections;

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

mc_Points mc_adjustWeightsThroughObservation(mc_Points currentPoints,
		TagDetection* tagDetections, Tag2DVector tags) {
	mc_Points result;
	TagExpectation* tagsExpected;
	int created;
	double p;

	for (int i = 0; i < mc_nSamples; ++i) {

		result.points[i] = currentPoints.points[i];
		tagsExpected = NULL;

		created = getExpectedTagDetections(&tagsExpected, result.points[i].x,
				result.points[i].y, result.points[i].theta, tags);

		p = mc_getSimilarityProbability(tagDetections, tagsExpected,
				result.points[i], created);
		if (i == 0) {
			//	mc_printTagDetections(tagDetections);
		}

		result.weights[i] = currentPoints.weights[i] * p;
	}
	return result;
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

int stepIndex = 0;
mc_Points mc_cycleMain(mc_Points currentPoints, double odo_Position[3],
		double old_odo[3], double covariance[3][3],
		TagDetection* tagDetections, Tag2DVector tags) {

	int sprintfResult;
	char outputFile[128];

	stepIndex++;

	sprintfResult = sprintf(outputFile, "~/simulation/results/initStep%d.m",
			stepIndex);

	mc_printSamples(outputFile, "w", NULL, currentPoints);

	mc_Points result = mc_generateNextOdometryPoints(currentPoints,
			odo_Position, old_odo, covariance);

	sprintf(outputFile, "~/simulation/results/afterOdometry%d.m", stepIndex);

	mc_printSamples(outputFile, "w", NULL, result);

	result = mc_cycleMainNoMovement(result, tagDetections, tags);

	return result;
}

//the main Monte Carlo cycle without the odometry step
// <- outputFile = file for debugging output
// <- currentPoints
// <- sensorModel
// <- tagDetections = the detections received in reality by the antennas
// <- number_of_detected_tags = the length of the former array
// <- tags
// <- tagmap
// -> the new points after a Monte Carlo cycle

mc_Points mc_cycleMainNoMovement(mc_Points currentPoints,
		TagDetection* tagDetections, Tag2DVector tags) {

	char outputFile[128];
	mc_Points result = mc_copyPoints(currentPoints);
	//mc_printSamples("Before observation:",result);


	result = mc_adjustWeightsThroughObservation(result, tagDetections, tags);
	//mc_printSamples("After observation:",result);

	sprintf(outputFile, "~/simulation/results/afterWeight%d.m", stepIndex);
	mc_printSamples(outputFile, "w", NULL, result);

	mc_normalize(&result);

	sprintf(outputFile, "~/simulation/results/afterNormalize%d.m", stepIndex);
	mc_printSamples(outputFile, "w", NULL, result);

	result = mc_generateNextUniformExtractedPoints2(&result);

	sprintf(outputFile, "~/simulation/results/afterResample%d.m", stepIndex);
	mc_printSamples(outputFile, "w", NULL, result);

	return result;
}
