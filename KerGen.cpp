// @author Merve Asiler

#include "KerGen.h"
#include "BaseGeoOpUtils.h"
#include "sdlp.h"
#include "CGALUtils.h"

#include <cinolib/predicates.h>

using namespace cinolib;

//constexpr double BIG_EPSILON = 1e-12;

KerGen::KerGen(const Mesh& hostMesh) :
	KernelExpansion(hostMesh, true) {

}

KerGen::~KerGen() {

}

void KerGen::expandKernel() {

	double point[3];
	if (this->initialPoint == NULL)
		return;		// NOT STAR-SHAPED!
	for (int i = 0; i < 3; i++)
		point[i] = this->initialPoint[i];

	vector<double> scalarsVector = initialize(point);
	findTheClosestHalfSpace3(point, scalarsVector);
	//int theClosestId1 = findTheClosestHalfSpace(point, scalarsVector);
	//int theClosestId2 = findTheClosestHalfSpace(point, theClosestId1);

	int number_of_kernel_faces = 0;
	int number_of_kernel_edges = 0;

	for (bool any_white_left = false; ; any_white_left = false) {
		for (int i = 0; i < halfSpaceSet.size(); i++) {
			if (isKernelFace[i] == ProcessColor::WHITE) {
				while (!edgePartners[i].empty()) {
					// initializations
					EdgePartnerTriple& ept = edgePartners[i].front();
					int partnerId = ept.partnerId;
					double edgeDirection[3] = { ept.edgeDirection[0], ept.edgeDirection[1], ept.edgeDirection[2] };
					double startPoint[3] = { ept.startPoint[0], ept.startPoint[1], ept.startPoint[2] };
					int startPointId = ept.startPointId;
					int backPlaneId = ept.backPlaneId;
					edgePartners[i].pop();

					// find the corner on the given edge
					int previousNumOfVerts = kernel.getNumOfVerts();
					double kernelVertex[3];
					vector<int> theClosestIds = findTheClosestHalfSpace(startPointId, edgeDirection, i, partnerId, backPlaneId, kernelVertex);

					if (kernel.getNumOfVerts() > previousNumOfVerts || isTripleSame(kernelVertex, startPoint, BIG_EPSILON))	// remove the second part ***
						orderTheFaces(i, partnerId, theClosestIds, kernelVertex, edgeDirection);
						
					number_of_kernel_edges++;
				}
				any_white_left = true;
				isKernelFace[i] = ProcessColor::GREEN;
				number_of_kernel_faces++;
				break;
			}
		}
		if (!any_white_left)
			break;
	}

	//cout << "[number of kernel edges: " << number_of_kernel_edges << "], number of kernel faces: " << number_of_kernel_faces << "]" << endl;

	// checkKernelForNonKernelVertices();

	/*
	if (kernel.getNumOfVerts() > 0)
		kernel = computeConvexHull(kernel.getAllVerts());
	*/

}

vector<double> KerGen::initialize(double* point) {

	double* scalarsArray = findValueVectorSatisfiedByPoint(point, halfSpaceSet);
	vector<double> scalarsVector;
	for (int i = 0; i < halfSpaceSet.size(); i++)
		scalarsVector.push_back(scalarsArray[i]);
	delete[] scalarsArray;

	for (int i = 0; i < halfSpaceSet.size(); i++) {
		edgePartners.push_back(queue<EdgePartnerTriple>());
		isKernelFace.push_back(ProcessColor::RED);
		edgePartnerIds.push_back(vector<int>());
	}

	double avgLength = 0;
	for (int e = 0; e < hostMeshptr->getNumOfEdges(); e++) {
		Edge edge = hostMeshptr->getEdge(e);
		Vertex v1 = hostMeshptr->getVertex(edge.endVerts[0]);
		Vertex v2 = hostMeshptr->getVertex(edge.endVerts[1]);
		edge.computeLength(&v1, &v2);
		avgLength += edge.length;
	}
	avgLength = avgLength / hostMeshptr->getNumOfEdges();
	//cout << avgLength << endl;
	
	BIG_EPSILON = avgLength * 1e-11;
	//BIG_EPSILON = avgLength * 1e-8;

	return scalarsVector;
}

int KerGen::findTheClosestHalfSpace2(double* point, vector<double>& scalarsVector) {

	double theClosestDistance = numeric_limits<double>::infinity();
	vector<int> theClosestIds;
	for (int i = 0; i < halfSpaceSet.size(); i++) {
		if (fabs(scalarsVector[i]) <= theClosestDistance) {
			if (fabs(scalarsVector[i]) < theClosestDistance)
				theClosestIds.clear();
			theClosestIds.push_back(i);
			theClosestDistance = fabs(scalarsVector[i]);
		}
	}

	int theClosestId = theClosestIds[0];
	scalarsVector.clear();
	double* direction = halfSpaceSet[theClosestId].ABCD; 
	
	/********************* MAKE ROBUST *********************/
	while (true) {

		double newpoint[3];
		for (int i = 0; i < 3; i++)
			newpoint[i] = point[i] + direction[i] * theClosestDistance;

		// check if the result is robust
		int is_robust = -1;
		for (int i = 0; i < halfSpaceSet.size(); i++) {
			double d = orient3d(halfSpaceSet[i].point1, halfSpaceSet[i].point2, halfSpaceSet[i].point3, newpoint);
			if (fabs(d) < BIG_EPSILON)
				continue; // return INTERSECT;
			else if (d > 0)
				continue; // return ABOVE;
			else {
				is_robust = i;
				cout << "d: " << d << endl;
				break; // return BELOW;
			}
		}

		if (is_robust < 0) {
			for (int i = 0; i < 3; i++)
				point[i] = newpoint[i];
			return theClosestId;
		}

		Line ray(point, direction);
		theClosestDistance = findLinePlaneIntersection(ray, halfSpaceSet[is_robust]);
		theClosestId = is_robust;
		cout << theClosestDistance << " " << theClosestId << endl;

	}
	/********************* MAKE ROBUST *********************/

}

int KerGen::findTheClosestHalfSpace2(double* point, int id) {

	double theClosestDistance = numeric_limits<double>::infinity();
	int theClosestId = -1;
	double theClosestDirection[3], intersectionLineDirection[3];
	for (int s = 0, i = 0, bound = id; s < 2; s++) {
		for (; i < bound; i++) {
			if (isTripleSame(halfSpaceSet[id].ABCD, halfSpaceSet[i].ABCD, BIG_EPSILON))
				continue;
			double lineDirection[3], rayDirection[3];
			crossProduct(halfSpaceSet[i].ABCD, halfSpaceSet[id].ABCD, lineDirection);
			normalize(lineDirection);
			crossProduct(halfSpaceSet[id].ABCD, lineDirection, rayDirection);
			normalize(rayDirection);
			Line ray(point, rayDirection);
			double t = findLinePlaneIntersection(ray, halfSpaceSet[i]);
			if (t < theClosestDistance) {
				theClosestId = i;
				theClosestDistance = t;
				for (int k = 0; k < 3; k++) {
					theClosestDirection[k] = rayDirection[k];
					intersectionLineDirection[k] = lineDirection[k];
				}
			}
		}
		i = id + 1;
		bound = halfSpaceSet.size();
	}

	double direction[3];
	for (int i = 0; i < 3; i++)
		direction[i] = theClosestDirection[i];
		//point[i] = point[i] + theClosestDirection[i] * theClosestDistance;

	/********************* MAKE ROBUST *********************/
	while (true) {

		double newpoint[3];
		for (int i = 0; i < 3; i++)
			newpoint[i] = point[i] + direction[i] * theClosestDistance;

		// check if the result is robust
		int is_robust = -1;
		for (int i = 0; i < halfSpaceSet.size(); i++) {
			double d = orient3d(halfSpaceSet[i].point1, halfSpaceSet[i].point2, halfSpaceSet[i].point3, newpoint);
			if (fabs(d) < BIG_EPSILON)
				continue; // return INTERSECT;
			else if (d > 0)
				continue; // return ABOVE;
			else {
				is_robust = i;
				cout << "d: " << d << endl;
				break; // return BELOW;
			}
		}

		if (is_robust < 0) {
			for (int i = 0; i < 3; i++)
				point[i] = newpoint[i];
			crossProduct(halfSpaceSet[theClosestId].ABCD, halfSpaceSet[id].ABCD, intersectionLineDirection);
			normalize(intersectionLineDirection);
			break;
		}

		Line ray(point, direction);
		theClosestDistance = findLinePlaneIntersection(ray, halfSpaceSet[is_robust]);
		theClosestId = is_robust;
		cout << theClosestDistance << " " << theClosestId << endl;

	}
	/********************* MAKE ROBUST *********************/

	double test_point[3];
	for (int k = 0; k < 3; k++)
		test_point[k] = point[k] + intersectionLineDirection[k];

	vector<int> parentIdsForNewVertex;
	bool edge_direction_proved = false;
	for (int i = 0; i < halfSpaceSet.size(); i++) {
		double d = orient3d(halfSpaceSet[i].point1, halfSpaceSet[i].point2, halfSpaceSet[i].point3, point);
		if (fabs(d) < BIG_EPSILON) {
			parentIdsForNewVertex.push_back(i);

			if (!edge_direction_proved) {
				double d = orient3d(halfSpaceSet[i].point1, halfSpaceSet[i].point2, halfSpaceSet[i].point3, test_point);
				if (fabs(d) < BIG_EPSILON)
					continue;
				else {
					edge_direction_proved = true;
					if (d < 0) {
						for (int k = 0; k < 3; k++)
							intersectionLineDirection[k] = -intersectionLineDirection[k];
					}
				}
			}

		}
	}
	vertexParentIds.push_back(parentIdsForNewVertex);
	kernel.addVertex(point[0], point[1], point[2]);

	edgePartners[id].push(EdgePartnerTriple(theClosestId, intersectionLineDirection, point, 0, -1));
	edgePartnerIds[id].push_back(theClosestId);
	edgePartnerIds[theClosestId].push_back(id);
	isKernelFace[id] = ProcessColor::WHITE;
	isKernelFace[theClosestId] = ProcessColor::WHITE;

	return theClosestId;

}


int KerGen::findTheClosestHalfSpace(double* point, vector<double>& scalarsVector) {

	double theClosestDistance = numeric_limits<double>::infinity();
	vector<int> theClosestIds;
	for (int i = 0; i < halfSpaceSet.size(); i++) {
		if (fabs(scalarsVector[i]) <= theClosestDistance) {
			if (fabs(scalarsVector[i]) < theClosestDistance)
				theClosestIds.clear();
			theClosestIds.push_back(i);
			theClosestDistance = fabs(scalarsVector[i]);
		}
	}

	int theClosestId = theClosestIds[0];
	scalarsVector.clear();
	for (int i = 0; i < 3; i++)
		point[i] = point[i] + halfSpaceSet[theClosestId].ABCD[i] * theClosestDistance;
	return theClosestId;
}

int KerGen::findTheClosestHalfSpace(double* point, int id) {

	double theClosestDistance = numeric_limits<double>::infinity();
	int theClosestId = -1;
	double theClosestDirection[3], intersectionLineDirection[3];
	for (int s = 0, i = 0, bound = id; s < 2; s++) {
		for (; i < bound; i++) {
			if (isTripleSame(halfSpaceSet[id].ABCD, halfSpaceSet[i].ABCD, BIG_EPSILON))
				continue;
			double lineDirection[3], rayDirection[3];
			crossProduct(halfSpaceSet[i].ABCD, halfSpaceSet[id].ABCD, lineDirection);
			normalize(lineDirection);
			crossProduct(halfSpaceSet[id].ABCD, lineDirection, rayDirection);
			normalize(rayDirection);
			Line ray(point, rayDirection);
			double t = findLinePlaneIntersection(ray, halfSpaceSet[i]);
			if (t < theClosestDistance) {
				theClosestId = i;
				theClosestDistance = t;
				for (int k = 0; k < 3; k++) {
					theClosestDirection[k] = rayDirection[k];
					intersectionLineDirection[k] = lineDirection[k];
				}
			}
		}
		i = id + 1;
		bound = halfSpaceSet.size();
	}

	for (int i = 0; i < 3; i++)
		point[i] = point[i] + theClosestDirection[i] * theClosestDistance;

	edgePartners[id].push(EdgePartnerTriple(theClosestId, intersectionLineDirection, point, 0, -1));
	edgePartnerIds[id].push_back(theClosestId);
	edgePartnerIds[theClosestId].push_back(id);
	isKernelFace[id] = ProcessColor::WHITE;
	isKernelFace[theClosestId] = ProcessColor::WHITE;

	vector<int> parentIdsForNewVertex;
	parentIdsForNewVertex.push_back(id);
	parentIdsForNewVertex.push_back(theClosestId);
	vertexParentIds.push_back(parentIdsForNewVertex);
	kernel.addVertex(point[0], point[1], point[2]);

	return theClosestId;

}


void KerGen::findTheClosestHalfSpace3(double* point, vector<double>& scalarsVector) {

	double theClosestDistance = numeric_limits<double>::infinity();
	vector<int> theClosestIds;
	for (int i = 0; i < halfSpaceSet.size(); i++) {
		if (fabs(scalarsVector[i]) <= BIG_EPSILON) {
			theClosestIds.push_back(i);
			theClosestDistance = fabs(scalarsVector[i]);
		}
	}

	scalarsVector.clear();

	//cout << theClosestIds.size() << endl;
	int theClosestId1 = theClosestIds[0];
	int theClosestId2 = theClosestIds[1];

	for (int i = 0; i < theClosestIds.size(); i++) {
		if (isTripleSame(halfSpaceSet[theClosestId1].ABCD, halfSpaceSet[theClosestIds[i]].ABCD, BIG_EPSILON))
			continue;
		theClosestId2 = theClosestIds[i];
		break;
	}
	
	double lineDirection[3], intersectionLineDirection[3];
	crossProduct(halfSpaceSet[theClosestId2].ABCD, halfSpaceSet[theClosestId1].ABCD, lineDirection);
	normalize(lineDirection);
	for (int k = 0; k < 3; k++)
		intersectionLineDirection[k] = lineDirection[k];
		
	edgePartners[theClosestId1].push(EdgePartnerTriple(theClosestId2, intersectionLineDirection, point, 0, -1));
	edgePartnerIds[theClosestId1].push_back(theClosestId2);
	edgePartnerIds[theClosestId2].push_back(theClosestId1);
	isKernelFace[theClosestId1] = ProcessColor::WHITE;
	isKernelFace[theClosestId2] = ProcessColor::WHITE;

	vector<int> parentIdsForNewVertex;
	parentIdsForNewVertex.push_back(theClosestId1);
	parentIdsForNewVertex.push_back(theClosestId2);
	vertexParentIds.push_back(parentIdsForNewVertex);
	kernel.addVertex(point[0], point[1], point[2]);

}

vector<int> KerGen::findTheClosestHalfSpace(int vertexId, double* lineDirection, int lineParent1Id, int lineParent2Id, int backPlaneId, double* newpoint) {

	vector<int> theClosestIds;
	Line line(kernel.getVertex(vertexId).coords, lineDirection);

	bool newpoint_exists = false;

	for (int i = 0; i < halfSpaceSet.size(); i++) {
		bool previous_parent = false;
		for (int j = 0; j < vertexParentIds[vertexId].size(); j++) {
			int p = vertexParentIds[vertexId][j];
			if (i == p) {
				previous_parent = true;
				break;
			}
		}
		if (previous_parent)
			continue;

		if (!newpoint_exists) {
			double t = findLinePlaneIntersection(line, halfSpaceSet[i], BIG_EPSILON);
			if (t == numeric_limits<double>::infinity())
				continue;
			for (int k = 0; k < 3; k++)
				newpoint[k] = kernel.getVertex(vertexId).coords[k] + lineDirection[k] * t;
			
			newpoint_exists = true;
			/*
			for (int j = 0; j < vertexParentIds[vertexId].size(); j++) {
				int p = vertexParentIds[vertexId][j];
				double d = orient3d(halfSpaceSet[p].point1, halfSpaceSet[p].point2, halfSpaceSet[p].point3, newpoint);
				if (fabs(d) < BIG_EPSILON)
					continue;	// return INTERSECT;
				else if (d > 0) 
					continue;	// return ABOVE;
				else {
					newpoint_exists = false;
					theClosestIds.clear();
					break;		// return BELOW;
				}
			}
			*/

			if (backPlaneId >= 0) {
				double d = orient3d(halfSpaceSet[backPlaneId].point1, halfSpaceSet[backPlaneId].point2, halfSpaceSet[backPlaneId].point3, newpoint);
				if (fabs(d) < BIG_EPSILON)
					;	// return INTERSECT;
				else if (d > 0)
					;	// return ABOVE;
				else {
					newpoint_exists = false;
					theClosestIds.clear();
					;		// return BELOW;
				}
			}

			if (newpoint_exists) 
				theClosestIds.push_back(i);
	
		}
		else {
			double d = orient3d(halfSpaceSet[i].point1, halfSpaceSet[i].point2, halfSpaceSet[i].point3, newpoint);
			if (fabs(d) < BIG_EPSILON)
				theClosestIds.push_back(i);
			else if (d > 0)
				continue;
			else {
				double t = findLinePlaneIntersection(line, halfSpaceSet[i], BIG_EPSILON);
				if (t == numeric_limits<double>::infinity()) {	// remove*	***
					cout << "--o--\n";
					theClosestIds.clear();
					return theClosestIds;
					//continue;					
				}

				for (int k = 0; k < 3; k++)
					newpoint[k] = kernel.getVertex(vertexId).coords[k] + lineDirection[k] * t;

				theClosestIds.clear();
				theClosestIds.push_back(i);
			}
		}
	}

	if (theClosestIds.size() == 0)
		return theClosestIds;

	int startId = vertexId;
	bool previousVertex = false;
	for (int v = 0; v < kernel.getNumOfVerts(); v++)
		if (isTripleSame(kernel.getVertex(v).coords, newpoint, BIG_EPSILON)) {
			previousVertex = true;
			vertexId = v;
			break;
		}
	if (!previousVertex) {
		kernel.addVertex(newpoint[0], newpoint[1], newpoint[2]);
		vector<int> parentIdsForNewVertex;
		vertexParentIds.push_back(parentIdsForNewVertex);
		vertexId = kernel.getNumOfVerts() - 1;

		for (int j = 0; j < vertexParentIds[startId].size(); j++) {
			int p = vertexParentIds[startId][j];
			double d = orient3d(halfSpaceSet[p].point1, halfSpaceSet[p].point2, halfSpaceSet[p].point3, newpoint);
			if (fabs(d) < BIG_EPSILON)
				theClosestIds.push_back(p);
		}
		for (int i = 0; i < theClosestIds.size(); i++)
			vertexParentIds[vertexId].push_back(theClosestIds[i]);
	}


	kernel.addEdge(startId, vertexId);

	return theClosestIds;

}

void KerGen::orderTheFaces(int base_id, int partner_id, vector<int> next_partner_ids, double* startPoint, double* currentEdgeDirection) {

	vector<int> generators_all = { base_id, partner_id };
	for (int i = 0; i < next_partner_ids.size(); i++)
		generators_all.push_back(next_partner_ids[i]);
	
	// remove the repeating planes 
	vector<int> generators;
	while (true) {
		bool theSame = false;
		for (int i = generators_all.size() - 1, j = i - 1; j >= 0; j--) {
			if (isTripleSame(halfSpaceSet[generators_all[i]].ABCD, halfSpaceSet[generators_all[j]].ABCD, BIG_EPSILON)) {
				theSame = true;
				break;
			}
		}
		
		if (!theSame)
			generators.push_back(generators_all[generators_all.size() - 1]);

		generators_all.pop_back();
		if (generators_all.size() == 0)
			break;
	}

	//vector<double> list_of_lines;

	for (int i = 0; i < generators.size(); i++) {
		for (int j = i + 1; j < generators.size(); j++) {

			if (isRecordedEdge(generators[i], generators[j]))
				continue;

			double nextEdgeDirection[3];
			crossProduct(halfSpaceSet[generators[i]].ABCD, halfSpaceSet[generators[j]].ABCD, nextEdgeDirection);
			if (isZeroVector(nextEdgeDirection, BIG_EPSILON))
				continue;
			normalize(nextEdgeDirection);

			/*
			// check whether this line recorded before
			bool recorded_before = false;
			for (int l = 0; l < list_of_lines.size(); l += 3) {
				double recordedDirection[3] = { list_of_lines[l], list_of_lines[l + 1], list_of_lines[l + 2] };
				if (fabs(fabs(dotProduct(recordedDirection, nextEdgeDirection)) - 1) < BIG_EPSILON) {
					recorded_before = true;
					break;
				}
			}
			if (recorded_before)
				continue;
			for (int k = 0; k < 3; k++)
				list_of_lines.push_back(nextEdgeDirection[k]);
			*/

			double test_point[2][3];
			for (int k = 0; k < 3; k++) {
				test_point[0][k] = startPoint[k] + nextEdgeDirection[k];
				test_point[1][k] = startPoint[k] - nextEdgeDirection[k];
			}

			double d1 = orient3d(halfSpaceSet[generators[i]].point1, halfSpaceSet[generators[i]].point2, halfSpaceSet[generators[i]].point3, test_point[0]);
			double d2 = orient3d(halfSpaceSet[generators[i]].point1, halfSpaceSet[generators[i]].point2, halfSpaceSet[generators[i]].point3, test_point[1]);
			double d3 = orient3d(halfSpaceSet[generators[j]].point1, halfSpaceSet[generators[j]].point2, halfSpaceSet[generators[j]].point3, test_point[0]);
			double d4 = orient3d(halfSpaceSet[generators[j]].point1, halfSpaceSet[generators[j]].point2, halfSpaceSet[generators[j]].point3, test_point[1]);
			if (fabs(d1) < BIG_EPSILON && fabs(d2) < BIG_EPSILON && fabs(d3) < BIG_EPSILON && fabs(d4) < BIG_EPSILON)
				;
			//else if (d1 < 0 || d2 < 0 || d3 < 0 || d4 < 0)
			//	continue;
			//	cout << "ERROR!!!!!!!!!!!!!!!!! " << d1 << " " << d2 << " " << d3 << " " << d4 << "\n";


			bool is_line_valid = false;
			bool change_edge_direction = false;
			int backPlaneId = -1;

			
			for (int tp = 0; tp < 2; tp++) {
				bool is_point_valid = true;
				for (int g = 0; g < generators.size(); g++) {
					int gnr = generators[g];
					if (gnr == generators[i] || gnr == generators[j])
						continue;
					double d = orient3d(halfSpaceSet[gnr].point1, halfSpaceSet[gnr].point2, halfSpaceSet[gnr].point3, test_point[tp]);
					if (fabs(d) < BIG_EPSILON)
						continue;
					else if (d > 0)
						continue;	// return ABOVE;
					else {
						if (tp == 0)
							change_edge_direction = true;
						is_point_valid = false;
						backPlaneId = gnr;
						break;		// return BELOW;
					}
				}

				if (is_point_valid) {
					//if (is_line_valid)
					//	cout << "BOTH TRUE................\n";
					is_line_valid = true;
				}
			}
			

			/******/
			/*
			bool is_point_valid[2] = {true, true};
			for (int g = 0; g < generators.size(); g++) {
				int gnr = generators[g];
				if (gnr == generators[i] || gnr == generators[j])
					continue;
				bool is_back_plane_allowed = true, is_back_plane_candidate[2] = { false, false };
				for (int tp = 0; tp < 2; tp++) {
					double d = orient3d(halfSpaceSet[gnr].point1, halfSpaceSet[gnr].point2, halfSpaceSet[gnr].point3, test_point[tp]);
					if (fabs(d) < BIG_EPSILON)
						is_back_plane_allowed = false;
					else if (d > 0)
						;
					else {
						if (tp == 0)
							change_edge_direction = true;
						is_point_valid[tp] = false;
						is_back_plane_candidate[tp] = true;
					}
				}

				if (is_back_plane_candidate[0] && is_back_plane_candidate[1]) {
					backPlaneId = -1;
					break;
				}
				if ((is_back_plane_candidate[0] || is_back_plane_candidate[1]) && is_back_plane_allowed)
					backPlaneId = gnr;
			}
			if (is_point_valid[0] && is_point_valid[2])
				continue;
			if (is_point_valid[0] || is_point_valid[2])
				is_line_valid = true;
			*/
			/******/

			if (backPlaneId < 0)
				continue;
			if (is_line_valid) {
				if (change_edge_direction) {
					for (int k = 0; k < 3; k++)
						nextEdgeDirection[k] = -nextEdgeDirection[k];
				}

				edgePartners[generators[i]].push(EdgePartnerTriple(generators[j], nextEdgeDirection, startPoint, kernel.getNumOfVerts() - 1, backPlaneId));
				//edgePartners[generators[j]].push(EdgePartnerTriple_Ideal(generators[i], nextEdgeDirection, startPoint, kernel.getNumOfVerts() - 1, backPlaneId));

				edgePartnerIds[generators[i]].push_back(generators[j]);
				edgePartnerIds[generators[j]].push_back(generators[i]);
				//if (isKernelFace[generators[i]] != ProcessColor_Ideal::GREEN)
					isKernelFace[generators[i]] = ProcessColor::WHITE;
				//if (isKernelFace[generators[j]] != ProcessColor_Ideal::GREEN)
					isKernelFace[generators[j]] = ProcessColor::WHITE;
			}

		}
	}

}

bool KerGen::isRecordedEdge(int hs1_id, int hs2_id) {

	for (int j = 0; j < edgePartnerIds[hs1_id].size(); j++)
		if (edgePartnerIds[hs1_id][j] == hs2_id)
			return true;
	return false;

}


