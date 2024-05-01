// @author Merve Asiler

#pragma once

#include "Mesh.h"
#include "BasicGeometricElements.h"
#include <queue>


class KernelExpansion {

protected:
	double* initialPoint = nullptr;
	double* extremeCorners[2] = { nullptr, nullptr };
	double* extremePoints[6] = { nullptr, nullptr, nullptr, nullptr, nullptr, nullptr };
	const Mesh* hostMeshptr;
	Mesh kernel;
	vector<HalfSpace> halfSpaceSet;
	double EPSILON_3 = 3 * EPSILON;

	bool isInKernel(double* scalarVector);
	vector<int> detectExcluderHalfSpaces(double* scalarVector);

public:
	KernelExpansion(const Mesh& hostMesh, bool defineInitialPoint);
	~KernelExpansion();
	Mesh& getKernel();
	double* getInitialKernelPoint();
	vector<HalfSpace>& getHalfSpaceSet();
	virtual void expandKernel() = 0;
	void checkKernelForNonKernelVertices();
	void checkKernelForIrregularTriangles();
};
