/*
 * MonteCarloPrintFunctions.cc
 *
 *  Created on: Nov 23, 2009
 *      Author: acodas
 */

#include "MonteCarloPrintFunctions.h"

//@tested
//prints a mc_Point
void mc_printPoint(mc_Point p) {
	printf("{x=%g, y=%g, theta=%g}", p.x, p.y, p.theta);
}

//@tested
//prints the matrix A
void mc_printMatrix(double A[3][3]) {
	for (int i = 0; i < 3; ++i) {
		for (int j = 0; j < 3; ++j) {
			printf("%g ", A[i][j]);
		}
		printf("\n");
	}
}

//prints the current samples with an introduction text
void mc_printSamples(const char* introduction, mc_Points currentPoints) {
	int numberPrinting = mc_nSamples;//3;
	if (introduction != NULL) {
		printf("%s\n", introduction);
	}

	if (numberPrinting > mc_nSamples) {
		numberPrinting = mc_nSamples;
	}

	for (int i = 0; i < numberPrinting; ++i) {
		printf("\t(%10.5lf,%10.5lf,%10.5lf) - %g\n", currentPoints.points[i].x,
				currentPoints.points[i].y, currentPoints.points[i].theta,
				currentPoints.weights[i]);
	}
}

//same as the former, but in a file
void mc_printSamples(const char OUTPUT_FILE_NAME[], const char fileOption[],const char* introduction, mc_Points currentPoints){

	int numberPrinting = mc_nSamples;

	FILE* outputFile = fopen(OUTPUT_FILE_NAME, fileOption);

	if (introduction != NULL) {
		fprintf(outputFile, "%s\n", introduction);
	}

	if (numberPrinting > mc_nSamples) {
		numberPrinting = mc_nSamples;
	}
	fprintf(outputFile, "m = [");
	int i;
	for (i = 0; i < numberPrinting - 1; ++i) {
		fprintf(outputFile, "\n\t%lf,\t%lf,\t%lf,\t%.20lf;",
				currentPoints.points[i].x, currentPoints.points[i].y,
				currentPoints.points[i].theta, currentPoints.weights[i]);
	}
	fprintf(outputFile, "\n\t%lf,\t%lf,\t%lf,\t%.20lf];",
			currentPoints.points[i].x, currentPoints.points[i].y,
			currentPoints.points[i].theta, currentPoints.weights[i]);
	fprintf(outputFile, "\nparticules = struct('pos',m);\n");
	fprintf(outputFile, "simulations = [simulations particules];\n");
	fclose(outputFile);
}
//prints the current samples with an introduction text


//prints tag detections
void mc_printTagDetections(TagDetection* tagDetections) {
	TagDetection* aux = tagDetections;
	int tagNum = -2;
	while (aux != 0) {

		if (tagNum == tagNumber(aux->tagid)) {
			printf(" %d", aux->antenna);
		} else {
			tagNum = tagNumber(aux->tagid);
			printf("\nTagDetection %d %s - antennas %d", tagNum, aux->tagid,
					aux->antenna);
		}

		aux = aux->next;
	}
	printf("\n");
}

void mc_printTagsExpected(TagExpectation* tagsExpected) {
	TagExpectation* aux = tagsExpected;
	int tagNum;
	while (aux != 0) {

		if (tagNum == tagNumber(aux->tagid)) {
			printf("\n%s antenna %d - probability %lf ", aux->tagid,
					aux->antenna, aux->probability);
		} else {
			tagNum = tagNumber(aux->tagid);
			printf("\nTagExpectation %d %s\n", tagNum, aux->tagid);
			printf("\n%s antenna %d - probability %lf ", aux->tagid,
					aux->antenna, aux->probability);
		}

		aux = aux->next;
	}
	printf("\n");

}

