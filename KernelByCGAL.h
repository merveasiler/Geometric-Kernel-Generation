// @author Merve Asiler

#pragma once

#include "KernelExpansion.h"

class KernelByCGAL : public KernelExpansion {

	vector<double*> halfSpaceCoeffs;

public:
	KernelByCGAL(const Mesh& hostMesh);
	~KernelByCGAL();
	void expandKernel();

};


