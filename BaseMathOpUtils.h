// @author Merve Asiler

#pragma once

#include <cmath>
#include <vector>
#include <tuple>

#define EPSILON 1e-12
#define PI 3.14159265

using namespace std;

class NOT_VALID : public exception {
public:
    virtual const char* what() const throw() {
        return "Not a valid mathematical operation, value, object, or etc.!";
    }
};

double computeLength(const double* vect);

void normalize(double* vect);

double* crossProduct(const double* vect1, const double* vect2);

void crossProduct(const double* vect1, const double* vect2, double* resultVect);

double dotProduct(const double* vect1, const double* vect2);

double* sumVects(const double* vect1, const double* vect2);

double* diffVects(const double* vect1, const double* vect2);

void multVect(const double* vect, double coeff, double* output);

double findCosAngleBetween(const double* vect1, const double* vect2);

double computeDeterminant3x3(double M[3][3]);
