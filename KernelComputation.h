// @author Merve Asiler

#pragma once
#pragma comment(lib, "boost_filesystem-vc140-mt.lib")

#include <string>
#include <vector>
using namespace std;

class Mesh;
class KernelExpansion;

void ComputeKernel(string meshName, string outputName, string algoType, string drawType);

void ComputeBatchKernel(string inputFolderName, string outputFolderName, string algoType);

Mesh Run(vector<KernelExpansion*>& kernelExpansions, Mesh& mesh, std::ofstream& outputFile, double& elapsedTime, string algoType);

void CompareKernelQuality(Mesh groundTruth, Mesh kernel, string algoType, std::ofstream& outputFile, double& volDiffPercentage, double* hausdorffDistances);

void FindKernelPoint_SDLP(string meshName);

void DoVisualComparisonOfAlgos(string meshName);


