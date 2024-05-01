// @author Merve Asiler

#include "BaseGeoOpUtils.h"
#include <iostream>
#include <Eigen/Dense>

//constexpr double _EPSILON_ = 1e-12;

int findPeakVertexID(Triangle* triangle, Edge* edge) {
	for (unsigned int i = 0; i < 3; i++) {
		if (triangle->corners[i] == edge->endVerts[0] || triangle->corners[i] == edge->endVerts[1])
			continue;
		return triangle->corners[i];
	}
	return INVALID_TRIANGLE;	// "not found" is not a possible case

}

double* findLine2LineIntersection(Line* line1, Line* line2) {	// Assume that they have a certain intersection point
	Eigen::Matrix2d A;
	A << line2->directionVector[0], -line1->directionVector[0],
		line2->directionVector[1], -line1->directionVector[1];
	Eigen::Vector2d b;
	b << line1->point[0] - line2->point[0],
		line1->point[1] - line2->point[1];
	Eigen::Vector2d x = A.colPivHouseholderQr().solve(b);

	double* point = new double[3];
	for (int i = 0; i < 3; i++)
		point[i] = line1->point[i] + line1->directionVector[i] * x[1];
	return point;
}

double findLinePlaneIntersection(Line& line, Plane& plane) {
	/* Assume they intersect at q = p + v*t (the rhs is parametric line equation)
	   A * q.x + B * q.y + C * q.z + D = 0	(comes from the plane equation)
	*/
	
	double midProduct = dotProduct(plane.ABCD, line.point);
	double numerator = midProduct + plane.ABCD[3];
	double denominator = dotProduct(plane.ABCD, line.directionVector);
	
	if (fabs(denominator) < 3 * EPSILON)
		return numeric_limits<double>::infinity();
	return -numerator / denominator;
}

double findLinePlaneIntersection(Line& line, Plane& plane, double _EPSILON_) {
	/* Assume they intersect at q = p + v*t (the rhs is parametric line equation)
	   A * q.x + B * q.y + C * q.z + D = 0	(comes from the plane equation)
	*/

	double midProduct = dotProduct(plane.ABCD, line.point);
	double numerator = midProduct + plane.ABCD[3];
	double denominator = dotProduct(plane.ABCD, line.directionVector);

	if (fabs(denominator) < 3 * _EPSILON_)
		return numeric_limits<double>::infinity();
	return -numerator / denominator;
}

double findRayTriangleIntersection(Line* ray, TriangleWithVerts* triangleWV) {

	triangleWV->computeNormal(triangleWV->vertices[0], triangleWV->vertices[1], triangleWV->vertices[2]);
	Plane* plane = new Plane(triangleWV->vertices[0]->coords, triangleWV->normal);

	double coeff = findLinePlaneIntersection(*ray, *plane);
	delete plane;
	if (isinf(coeff)) {
		return numeric_limits<double>::infinity();
	}

	double Q[3];	// intersection point
	for (int k = 0; k < 3; k++)
		Q[k] = ray->point[k] + coeff * ray->directionVector[k];

	// compute barycentric coordinates of Q
	double* baryCoords = triangleWV->computeBarycentricCoords(Q);
	if (baryCoords == NULL)
		return numeric_limits<double>::infinity();
	delete[] baryCoords;
	return coeff;

}

Line* find2PlaneIntersection(Plane planes[2]) {

	double dirVector[3];
	crossProduct(planes[0].ABCD, planes[1].ABCD, dirVector);
	normalize(dirVector);

	double point[3];
	double coeffs[2][2] = { {planes[0].ABCD[1], planes[0].ABCD[2]}, {planes[1].ABCD[1], planes[1].ABCD[2]} };
	double det = coeffs[0][0] * coeffs[1][1] - coeffs[0][1] * coeffs[1][0];
	if (abs(det) < 2 * EPSILON) {
		point[0] = (planes[0].ABCD[1] * dirVector[1] + planes[0].ABCD[2] * dirVector[2] + planes[0].ABCD[3]) / (-planes[0].ABCD[0]);
		point[1] = dirVector[1];
		point[2] = dirVector[2];
	}
	else {
		double A[2][2];
		A[0][0] = coeffs[1][1] / det;
		A[1][1] = coeffs[0][0] / det;
		A[0][1] = -coeffs[0][1] / det;
		A[1][0] = -coeffs[1][0] / det;
		double B[2];
		B[0] = -planes[0].ABCD[3];
		B[1] = -planes[1].ABCD[3];
		
		for (int i = 0; i < 2; i++)
			point[i+1] = A[i][0] * B[0] + A[i][1] * B[1];
		point[0] = 0;

	}

	Line* line = new Line(point, dirVector);
	return line;
}

double* find3PlaneIntersection(Plane planes[3]) {
	// Cramer's rule
	
	double ABC[3][3] = { {planes[0].ABCD[0], planes[0].ABCD[1], planes[0].ABCD[2]},
						 {planes[1].ABCD[0], planes[1].ABCD[1], planes[1].ABCD[2]},
						 {planes[2].ABCD[0], planes[2].ABCD[1], planes[2].ABCD[2]} };

	double detABC = computeDeterminant3x3(ABC);
	if (abs(detABC) < 3*EPSILON*EPSILON)
		return NULL;

	double ABC_x[3][3], ABC_y[3][3], ABC_z[3][3];
	for (int i = 0; i < 3; i++) {
		for (int j = 0; j < 3; j++) {
			ABC_x[i][j] = ABC[i][j];
			ABC_y[i][j] = ABC[i][j];
			ABC_z[i][j] = ABC[i][j];
		}
		ABC_x[i][0] = -planes[i].ABCD[3];
		ABC_y[i][1] = -planes[i].ABCD[3];
		ABC_z[i][2] = -planes[i].ABCD[3];
	}

	double* xyz = new double[3];
	double dets[3] = { computeDeterminant3x3(ABC_x), computeDeterminant3x3(ABC_y), computeDeterminant3x3(ABC_z) };

	for (int i = 0; i < 3; i++)
		xyz[i] = dets[i] / detABC;

	return xyz;
}

void projectVertexOnPlane(Plane plane, double* vertex, double* projectionpoint) {
	double A = plane.ABCD[0];
	double B = plane.ABCD[1];
	double C = plane.ABCD[2];
	double D = plane.ABCD[3];
	double Px = vertex[0];
	double Py = vertex[1];
	double Pz = vertex[2];

	/* If the projection coordinates are Q=(x,y,z),
		then Q - P = t*N, where N is the plane normal.
		In other words:
		(x - Px, y - Py, z - Pz) = t * (Nx, Ny, Nz)
		This results in 3 equations:
		x = Px + t * Nx	
		y = Py + t * Ny
		z = Pz + t * Nz
		Put Q=(x,y,z) into the plane equation:
		A * (Px + t * Nx) + B * (Py + t * Ny) + C * (Pz + t * Nz) + D = 0
		=> t = -(A*Px + B*Py + C*Pz + D) / (A*Nx + B*Ny + C*Nz)
		Do not forget that <Nx, Ny, Nz> = <A, B, C>.
	*/
	double t = -(A * Px + B * Py + C * Pz + D) / (A * A + B * B + C * C);
	for (int i = 0; i < 3; i++)
		projectionpoint[i] = vertex[i] + t * plane.ABCD[i];

	/*
	// TO-DO: What if either one of A,B,C is 0?
	double tempProductY = (A * Py - B * Px);
	double tempProductZ = (A * Pz - C * Px);

	point[0] = -(D * A + (B * tempProductY + C * tempProductZ)) / (A * A + B * B + C * C);
	point[1] = (tempProductY + B * point[0]) / A;
	point[2] = (tempProductZ + C * point[0]) / A;
	*/

}

/*
	Is the given 3D point inside the region enclosed by the given half space?
	returns 1 if the point is inside the region
	returns 0 if the point is on the boundary of the region
	returns -1 if the point is outside the region
*/
int isPointInRegion(const double* point, const HalfSpace* halfSpace) {
	double total = halfSpace->ABCD[3];
	for (unsigned int j = 0; j < 3; j++)
		total += halfSpace->ABCD[j] * point[j];
	if (abs(total) < EPSILON)
		return 0;	// on the boundary
	else if (halfSpace->isLargerThanZero && total >= 0)
		return 1;	// inside
	else if (!halfSpace->isLargerThanZero && total <= 0)
		return 1;	// inside
	else
		return -1;	// outside
}

double findClosestValueSatisfiedByPoint(const double point[3], const vector<HalfSpace>& halfSpaceContainer) {

	double closestValue = numeric_limits<double>::infinity();
	double furthestValue = -numeric_limits<double>::infinity();

	for (unsigned int i = 0; i < halfSpaceContainer.size(); i++) {
		HalfSpace halfSpace = halfSpaceContainer[i];
		
		double total = halfSpace.ABCD[3];
		for (unsigned int j = 0; j < 3; j++) {
			double mult = halfSpace.ABCD[j] * point[j];
			total += mult;
		}
		
		if ( (halfSpace.isLargerThanZero && total >= 0) || (!halfSpace.isLargerThanZero && total <= 0) ) {
			if (abs(total) - closestValue < 3*EPSILON)
				closestValue = abs(total);
		}
		else {
			if (furthestValue - abs(total) < 3*EPSILON)
				furthestValue = abs(total);
		}
	}

	if (furthestValue > 0)
		return furthestValue;
	return -closestValue;
}

double* findValueVectorSatisfiedByPoint(const double point[3], const vector<HalfSpace>& halfSpaceContainer) {
	
	double* valuesVector = new double[halfSpaceContainer.size()];

	for (unsigned int i = 0; i < halfSpaceContainer.size(); i++) {

		double total = halfSpaceContainer[i].ABCD[3];
		for (unsigned int j = 0; j < 3; j++) {
			double mult = halfSpaceContainer[i].ABCD[j] * point[j];
			total += mult;
		}

		valuesVector[i] = total;
	}

	return valuesVector;
}

void computeHalfSpacesFromTriangles(const vector<Triangle>& tris, const vector<Vertex>& verts, vector<HalfSpace>& halfSpaces) {
	
	for (int i = 0; i < tris.size(); i++) {
		Triangle tri = tris[i];
		HalfSpace halfSpace(verts[tri.corners[0]].coords, verts[tri.corners[1]].coords, verts[tri.corners[2]].coords, false);
		halfSpaces.push_back(halfSpace);
	}

}

vector<double*> computeHalfSpaceCoeffsFromTriangles(const vector<Triangle>& tris, const vector<Vertex>& verts) {

	vector<double*> halfSpaceCoeffs;
	vector<HalfSpace> halfSpaces;
	computeHalfSpacesFromTriangles(tris, verts, halfSpaces);

	for (int i = 0; i < halfSpaces.size(); i++) {
		double* coeffs = new double[4];
		for (int j = 0; j < 4; j++)
			coeffs[j] = halfSpaces[i].ABCD[j];
		halfSpaceCoeffs.push_back(coeffs);
	}
	halfSpaces.clear();
	
	return halfSpaceCoeffs;
}

double findAngleBetweenPlaneAndVector(const Plane* plane, const double* direction) {

	double cosAngleWithNormal = findCosAngleBetween(plane->ABCD, direction);
	double angleWithNormal = acos(cosAngleWithNormal);
	return abs(angleWithNormal - M_PI_2);

}

double* projectVectorOnPlane(const Plane* plane, const double* direction) {

	double q[3];	// represents the point obtained by going along the <direction> from <plane->point>
	for (int i = 0; i < 3; i++)
		q[i] = plane->point[i] + direction[i];
	
	double angle = findAngleBetweenPlaneAndVector(plane, direction);
	double height = computeLength(direction) * abs(sin(angle));	// height of q to the plane
	
	double p[3]; // projection of q on to the plane
	for (int i = 0; i < 3; i++)
		p[i] = q[i] - plane->ABCD[i] * height;

	double* projection = new double[3];	// projection of <direction> on to the <plane>
	for (int i = 0; i < 3; i++)
		projection[i] = p[i] - plane->point[i];

	return projection;
}

HalfPlane* projectHalfSpaceOnPlane(const Plane* basePlane, const HalfSpace* hs) {

	double* directionOfLine = crossProduct(basePlane->ABCD, hs->ABCD);	// direction of the line occuring with the intersection of two planes 
	// Check whether the two planes are parallel (or the same) or not
	bool is_parallel = true;
	for (int i = 0; i < 3; i++)	// if directionOfLine is <0, 0, 0>
		if (abs(directionOfLine[i]) > EPSILON)
			is_parallel = false;
	if (is_parallel)
		throw PARALLEL_PLANE_EXCEPTION();

	// Find a common point of two planes:
	Eigen::MatrixXd A(2,3);
	A << basePlane->ABCD[0], basePlane->ABCD[1], basePlane->ABCD[2],
		 hs->ABCD[0], hs->ABCD[1], hs->ABCD[2];
	Eigen::Vector2d b;
	b << -basePlane->ABCD[3], -hs->ABCD[3];
	Eigen::Vector3d x = A.colPivHouseholderQr().solve(b);
	double point[3] = {x[0], x[1], x[2]};

	// lastly, compute the normal of half plane (on the basePlane) and construct it
	double* normalOnPlane = projectVectorOnPlane(basePlane, hs->ABCD);
	normalize(normalOnPlane);
	HalfPlane* projection = new HalfPlane(point, directionOfLine, normalOnPlane, false);

	// clean-up
	delete[] directionOfLine;
	directionOfLine = nullptr;
	delete[] normalOnPlane;
	normalOnPlane = nullptr;

	return projection;

}

double* rotate3x1(double rotationMatrix[3][3], double element3x1[3]) {
	double* rotated3x1 = new double[3];
	for (int i = 0; i < 3; i++) {
		rotated3x1[i] = 0;
		for (int j = 0; j < 3; j++)
			rotated3x1[i] += rotationMatrix[i][j] * element3x1[j];
	}
	return rotated3x1;
}

bool isZeroVector(double* vect) {
	bool result = true;
	for (int i = 0; i < 3; i++)
		result = result && (fabs(vect[i]) < EPSILON ? true : false);
	return result;
}

bool isZeroVector(double* vect, double _EPSILON_) {
	bool result = true;
	for (int i = 0; i < 3; i++)
		result = result && (fabs(vect[i]) < _EPSILON_ ? true : false);
	return result;
}

bool isTripleSame(double* p1, double* p2) {
	
	if (fabs(p1[0] - p2[0]) < 2 * EPSILON && fabs(p1[1] - p2[1]) < 2 * EPSILON && fabs(p1[2] - p2[2]) < 2 * EPSILON)
		return true;
	return false;
}

bool isTripleSame(double* p1, double* p2, double _EPSILON_) {

	if (fabs(p1[0] - p2[0]) < 2 * _EPSILON_ && fabs(p1[1] - p2[1]) < 2 * _EPSILON_ && fabs(p1[2] - p2[2]) < 2 * _EPSILON_)
		return true;
	return false;
}
