// @author Merve Asiler

#pragma once

#include "BaseMathOpUtils.h"

#include <CGAL/Exact_predicates_exact_constructions_kernel.h>


struct Line {

	/* Line Equation is: "point + t * directionVector" where
		point and directionVector consist of x, y, z coordinates
		and t is the parameter.
	*/
	double point[3];
	double directionVector[3];

	Line() {};
	Line(double*, double*);
	~Line();
};

struct HalfPlane : Line {

	double normalOnPlane[3];		// normal vector of border line on the plane
	bool doesCoverNormalSide;	// which side of the plane line, is normalOnPlane side, or the other side?

	HalfPlane(double*, double*, double*, bool);
	~HalfPlane();
};

struct Plane {

		/* Plane Equation is: "Ax + By + Cz + D = 0" where
			A, B, C and D are ABCD[0], ABCD[1], ABCD[2], ABCD[3], resp.
			such that <A,B,C> is the normal vector,
			and D = -(<A,B,C> dotproduct point),
			and point is any point on the plane having x, y, z coordinates.
		*/
	int idx;
	double ABCD[3];
	double point[3];
	double point1[3], point2[3], point3[3];

	Plane(const double*, const double*,const double*);
	Plane(const double*, const double*);
	~Plane();
	void setId(int idx);
};

struct HalfSpace : Plane {

	bool isLargerThanZero;

	HalfSpace(const double*, const double*, const double*, bool);
	HalfSpace(const double*, const double*, bool);
	~HalfSpace();
};