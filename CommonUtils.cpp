// @author Merve Asiler

#include "CommonUtils.h"
#include "BaseMathOpUtils.h"
#include <iostream>
#include <fstream>
#include <sstream>

vector<string> split(string inputStr, string delimiter) {

	vector<string> pieces;
	string data = inputStr, subdata = "";

	while (data != "") {
		int ind = data.find_first_of(delimiter);
		
		if (ind < 0)
			subdata = data;
		else
			subdata = data.substr(0, ind);
		
		if (subdata != "")
			pieces.push_back(subdata);

		if (ind < 0)
			break;
		data = data.substr(ind + 1);
	}

	return pieces;

}

void fetchStatistics(string inpFileName, string outFileName, bool checkQuality) {

	string line;
	ifstream inpFile(inpFileName), curLinePtr;

	ofstream outFile_star, outFile_nonstar, outFile_volume, outFile_hausdorff;
	outFile_star.open(outFileName + "_stars.txt");
	outFile_nonstar.open(outFileName + "_nonstars.txt");
	outFile_volume.open(outFileName + "_volumes.txt");
	outFile_hausdorff.open(outFileName + "_hausdorffs.txt");

	if (inpFile.is_open()) {
		while (!inpFile.eof()) {

			getline(inpFile, line);
			int ind = line.find("Mesh: [faces: ");
			if (ind >= 0) {
				string faceNumberAsStr = line.substr(14, line.find("]"));
				int numOfFaces;
				sscanf_s(faceNumberAsStr.c_str(), "%d/", &numOfFaces);


				getline(inpFile, line);
				int ind_k = line.find("Kernel: [faces: ");

				getline(inpFile, line);
				string timeAsStr = line.substr(41, 41 + line.substr(41).find(" "));
				double time;
				sscanf_s(timeAsStr.c_str(), "%lf/", &time);

				if (ind_k >= 0) {	// star-shape
					outFile_star << numOfFaces << "\t\t\t" << round(time) << endl;


					if (checkQuality) {
						for (int l = 0; l < 3; l++)
							getline(inpFile, line);
						string volumeAsStr = line.substr(32, line.find("%"));
						double volume;
						sscanf_s(volumeAsStr.c_str(), "%lf/", &volume);
						outFile_volume << numOfFaces << "\t\t\t" << volume << endl;


						for (int l = 0; l < 4; l++)
							getline(inpFile, line);
						string hausdorffAsStr = line.substr(line.find(":") + 2, line.find(" ."));
						double hausdorff;
						sscanf_s(hausdorffAsStr.c_str(), "%lf/", &hausdorff);
						outFile_hausdorff << numOfFaces << "\t\t\t" << hausdorff << endl;
					}
				}
				else            // nonstar shape
					outFile_nonstar << numOfFaces << "\t\t\t" << round(time) << endl;
			}


		}

		inpFile.close();
	}

	outFile_star.close();
	outFile_nonstar.close();
	outFile_volume.close();
	outFile_hausdorff.close();

}

tuple<vector<vector<int>>, vector<vector<double>>> kMeans(vector<double> numberSet, int k) {

	vector<vector<double>> clusterVals;
	vector<vector<int>> clusterKeys;
	vector<double> centers;
	clusterVals.resize(k);
	clusterKeys.resize(k);
	
	// randomly initialize centers
	for (int i = 0; i < k; i++) {
		int ind = rand() % numberSet.size();
		if (find(centers.begin(), centers.end(), numberSet[ind]) == centers.end())
			centers.push_back(numberSet[ind]);
		else
			i--;
	}

	bool is_repeat = true;
	int counter = 0;

	while (is_repeat) {
		for (int i = 0; i < k; i++) {
			clusterVals[i].clear();
			clusterKeys[i].clear();
		}

		for (int i = 0; i < numberSet.size(); i++) {
			double minDistance = numeric_limits<double>::infinity();
			int clusterInd;
			for (int j = 0; j < k; j++) {
				if (abs(numberSet[i] - centers[j]) < minDistance) {
					minDistance = abs(numberSet[i] - centers[j]);
					clusterInd = j;
				}
			}
			clusterVals[clusterInd].push_back(numberSet[i]);
			clusterKeys[clusterInd].push_back(i);
		}

		is_repeat = false;
		counter++;
		cout << counter << endl;

		for (int i = 0; i < k; i++) {
			double sum = 0;
			for (int j = 0; j < clusterVals[i].size(); j++)
				sum += clusterVals[i][j];
			double newCenter = sum / clusterVals[i].size();
			if (abs(newCenter - centers[i]) >= EPSILON)
				is_repeat = true;
			centers[i] = newCenter;
		}
	}

	tuple< vector<vector<int>>, vector<vector<double>> > clusters = make_tuple(clusterKeys, clusterVals);
	return clusters;
}


