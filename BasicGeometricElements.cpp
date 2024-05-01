// @author Merve Asiler

#include "BasicGeometricElements.h"

#pragma region LINE

/*
	@param A point on the line:
	@param The direction vector of the line
*/
Line::Line(double* point, double* directionVector) {

	for (unsigned int i = 0; i < 3; i++) {
		this->point[i] = point[i];
		this->directionVector[i] = directionVector[i];
	}
}

Line::~Line() {

}

#pragma endregion

#pragma region HALFPLANE

HalfPlane::HalfPlane(double* point, double* directionVector, double* normalVector, bool doesCoverNormalSide) : Line(point, directionVector) {

	for (int i = 0; i < 3; i++)
		this->normalOnPlane[i] = normalVector[i];
	this->doesCoverNormalSide = doesCoverNormalSide;
}

HalfPlane::~HalfPlane() {

}

#pragma endregion

#pragma region PLANE

/*
	@param A point on the plane
	@param The normal vector of the plane
*/
Plane::Plane(const double* point, const double* normalVector) {

	for (unsigned int i = 0; i < 3; i++) {
		this->ABCD[i] = normalVector[i];
		this->point[i] = point[i];
	}

	this->ABCD[3] = -dotProduct(normalVector, point);
}

/*
	@param 3 points that construct the plane
*/
Plane::Plane(const double* point1, const double* point2, const double* point3) {

	for (unsigned int i = 0; i < 3; i++) {
		this->point1[i] = point1[i];
		this->point2[i] = point2[i];
		this->point3[i] = point3[i];
		
		this->point[i] = point1[i];
	}

	// compute normal
	double edgeVect1[3], edgeVect2[3];
	for (int i = 0; i < 3; i++) {
		edgeVect1[i] = point2[i] - point1[i];
		edgeVect2[i] = point3[i] - point2[i];
	}
	crossProduct(edgeVect1, edgeVect2, this->ABCD);
	normalize(this->ABCD);
	
	this->ABCD[3] = -dotProduct(this->ABCD, this->point);
}

Plane::~Plane() {

}

void Plane::setId(int idx) {
	
	this->idx = idx;
}

#pragma endregion

#pragma region HALFSPACE

HalfSpace::HalfSpace(const double* point, const double* normalVector, bool isLargerThanZero) : Plane(point, normalVector) {

	this->isLargerThanZero = isLargerThanZero;
}

HalfSpace::HalfSpace(const double* point1, const double* point2, const double* point3, bool isLargerThanZero) : Plane(point1, point2, point3) {

	this->isLargerThanZero = isLargerThanZero;
}

HalfSpace::~HalfSpace() {

}

#pragma endregion