// @author Merve Asiler

#include "BaseMathOpUtils.h"
#include <iostream>

double computeLength(const double* vect) {
	return sqrt((vect[0] * vect[0]) + (vect[1] * vect[1]) + (vect[2] * vect[2]));
}

void normalize(double* vect) {
	double length = computeLength(vect);
	for (int i = 0; i < 3; i++)
		vect[i] = vect[i] / length;
}

double* crossProduct(const double* vect1, const double* vect2) {
	double* resultVect = new double[3];
	resultVect[0] = vect1[1] * vect2[2] - vect1[2] * vect2[1];
	resultVect[1] = -vect1[0] * vect2[2] + vect1[2] * vect2[0];
	resultVect[2] = vect1[0] * vect2[1] - vect1[1] * vect2[0];
	return resultVect;
}

void crossProduct(const double* vect1, const double* vect2, double* resultVect) {
	resultVect[0] = vect1[1] * vect2[2] - vect1[2] * vect2[1];
	resultVect[1] = -vect1[0] * vect2[2] + vect1[2] * vect2[0];
	resultVect[2] = vect1[0] * vect2[1] - vect1[1] * vect2[0];
}

double dotProduct(const double* vect1, const double* vect2) {
	double dotProductOutput = 0.0;
	for (unsigned int i = 0; i < 3; i++)
		dotProductOutput += vect1[i] * vect2[i];
	return dotProductOutput;
}

double* sumVects(const double* vect1, const double* vect2) {
	double* output = new double[3];
	for (unsigned int i = 0; i < 3; i++)
		output[i] = vect1[i] + vect2[i];
	return output;
}

double* diffVects(const double* vect1, const double* vect2) {
	double* output = new double[3];
	for (unsigned int i = 0; i < 3; i++)
		output[i] = vect1[i] - vect2[i];
	return output;
}

void multVect(const double* vect, double coeff, double* output) {

	for (unsigned int i = 0; i < 3; i++)
		output[i] = vect[i] * coeff;
}

double findCosAngleBetween(const double* vect1, const double* vect2) {
	return dotProduct(vect1, vect2) / (computeLength(vect1) * computeLength(vect2));
}

double computeDeterminant3x3(double M[3][3]) {
	return	M[0][0] * (M[1][1] * M[2][2] - M[1][2] * M[2][1]) -
			M[0][1] * (M[1][0] * M[2][2] - M[1][2] * M[2][0]) +
			M[0][2] * (M[1][0] * M[2][1] - M[1][1] * M[2][0]);
}