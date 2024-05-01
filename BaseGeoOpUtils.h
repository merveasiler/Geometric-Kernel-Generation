// @author Merve Asiler

#pragma once

#define _USE_MATH_DEFINES
#include <exception>
#include "BasicMeshElements.h"
#include "BasicGeometricElements.h"

#define INVALID_TRIANGLE -1

class SKEW_ELEMENTS_EXCEPTION : public exception {
public:
    virtual const char* what() const throw() {
        return "The given elements do not intersect since they are skew!";
    }
};

class PARALLEL_PLANE_EXCEPTION : public exception {
public:
    virtual const char* what() const throw() {
        return "The given elements do not intersect since they have the parallel normals!";
    }
};

class NOT_ON_TRIANGLE : public exception {
public:
    virtual const char* what() const throw() {
        return "The given ray does not intersect the triangle!";
    }
};

int findPeakVertexID(Triangle* triangle, Edge* edge);

double* findLine2LineIntersection(Line* line1, Line* line2);        // assumes that they have a certain intersection point

double  findLinePlaneIntersection(Line& line, Plane& plane);         // returns the parameter for the line where it intersects with the plane

double  findLinePlaneIntersection(Line& line, Plane& plane, double _EPSILON_);       // epsilon is different

double findRayTriangleIntersection(Line* ray, TriangleWithVerts* triangleWV);  // returns the parameter for the ray where it intersects with the triangle

Line* find2PlaneIntersection(Plane planes[2]);                     // returns the line formed by the intersection of 2 non-parallel planes

double* find3PlaneIntersection(Plane planes[3]);                   // returns the point formed by the intersection of 3 non-parallel planes

void projectVertexOnPlane(Plane plane, double* vertex, double* projectionpoint);

int isPointInRegion(const double* point, const HalfSpace* halfSpace);

double findClosestValueSatisfiedByPoint(const double point[3], const vector<HalfSpace>& halfSpaceContainer);

double* findValueVectorSatisfiedByPoint(const double point[3], const vector<HalfSpace>& halfSpaceContainer);

void computeHalfSpacesFromTriangles(const vector<Triangle>& tris, const vector<Vertex>& verts, vector<HalfSpace>& halfSpaces);

vector<double*> computeHalfSpaceCoeffsFromTriangles(const vector<Triangle>& tris, const vector<Vertex>& verts);

double findAngleBetweenPlaneAndVector(const Plane* plane, const double* direction);

double* projectVectorOnPlane(const Plane* plane, const double* direction);

HalfPlane* projectHalfSpaceOnPlane(const Plane* basePlane, const HalfSpace* hs);

double* rotate3x1(double rotationMatrix[3][3], double element3x1[3]);

bool isZeroVector(double* vect);

bool isZeroVector(double* vect, double _EPSILON_);

bool isTripleSame(double* p1, double* p2);

bool isTripleSame(double* p1, double* p2, double _EPSILON_);
