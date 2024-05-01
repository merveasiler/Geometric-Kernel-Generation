// @author Merve Asiler

#include "KernelExpansion.h"
#include "BaseGeoOpUtils.h"
#include "sdlp.h"

KernelExpansion::KernelExpansion(const Mesh& hostMesh, bool defineInitialPoint) {

	this->hostMeshptr = &hostMesh;

	computeHalfSpacesFromTriangles(hostMeshptr->getAllTris(), hostMeshptr->getAllVerts(), this->halfSpaceSet);

	// FIND EXTREME KERNEL BOUNDING CORNERS BY USING ALL OF THE 6 EXTREME DIRECTIONS, TAKE THEIR AVERAGE
	extremeCorners[0] = new double[3];	extremeCorners[1] = new double[3];
	double extremeDirections[6][3] = { {-1, 0, 0}, {1, 0, 0}, {0, -1, 0}, {0, 1, 0}, {0, 0, -1}, {0, 0, 1} };
	for (int i = 0; i < 6; i++) {
		this->extremePoints[i] = nullptr;
		this->extremePoints[i] = sdlpMain(extremeDirections[i], halfSpaceSet);	// compute initial kernel point at the given extreme direction
		if (!this->extremePoints[i])
			return;
		this->extremeCorners[i % 2][int(i / 2)] = this->extremePoints[i][int(i / 2)];
	}

	if (defineInitialPoint) {
		this->initialPoint = new double[3]{ 0, 0, 0 };
		for (int i = 0; i < 3; i++)
			this->initialPoint[i] = this->extremePoints[0][i];
	}
	else
		this->initialPoint = nullptr;

}

KernelExpansion::~KernelExpansion() {

	if (initialPoint) {
		delete[] initialPoint;
		initialPoint = nullptr;
	}

	if (extremeCorners[0]) {
		delete[] extremeCorners[0];
		extremeCorners[0] = nullptr;
		delete[] extremeCorners[1];
		extremeCorners[1] = nullptr;
	}

	for (int i = 0; i < 6; i++) {
		if (extremePoints[i]) {
			delete[] extremePoints[i];
			extremePoints[i] = nullptr;
		}
	}

}

Mesh& KernelExpansion::getKernel() {
	return kernel;
}

double* KernelExpansion::getInitialKernelPoint() {
	return initialPoint;
}

vector<HalfSpace>& KernelExpansion::getHalfSpaceSet() {
	return halfSpaceSet;
}

bool KernelExpansion::isInKernel(double* scalarVector) {
	for (int i = 0; i < halfSpaceSet.size(); i++)
		if (scalarVector[i] > EPSILON_3)
			return false;	// out
	return true;		// in
}

vector<int> KernelExpansion::detectExcluderHalfSpaces(double* scalarVector) {
	vector<int> excluderIds;
	for (int i = 0; i < halfSpaceSet.size(); i++)
		if (scalarVector[i] > EPSILON_3)
			excluderIds.push_back(i);
	return excluderIds;
}

void KernelExpansion::checkKernelForNonKernelVertices() {

	int num_of_non_kernel_points = 0;
	for (int i = 0; i < kernel.getNumOfVerts(); i++) {
		if (findClosestValueSatisfiedByPoint(kernel.getVertex(i).coords, halfSpaceSet) > 3 * EPSILON ||
			findClosestValueSatisfiedByPoint(kernel.getVertex(i).coords, halfSpaceSet) > 3 * EPSILON ||
			findClosestValueSatisfiedByPoint(kernel.getVertex(i).coords, halfSpaceSet) > 3 * EPSILON) {
			cout << "non-kernel point" << endl;
			num_of_non_kernel_points++;
		}
	}
	cout << "num of non-kernel points is: " << num_of_non_kernel_points << endl;

}

void KernelExpansion::checkKernelForIrregularTriangles() {

	int num_of_irregular_triangles = 0;
	for (int i = 0; i < kernel.getNumOfTris(); i++) {
		if (kernel.getTriangle(i).corners[0] == kernel.getTriangle(i).corners[1] ||
			kernel.getTriangle(i).corners[1] == kernel.getTriangle(i).corners[2] ||
			kernel.getTriangle(i).corners[2] == kernel.getTriangle(i).corners[0]) {
			cout << "errorrrrr: triangle no: " << i << endl;
			num_of_irregular_triangles++;
		}
	}
	cout << "num of irregular triangles is: " << num_of_irregular_triangles << endl;

}


