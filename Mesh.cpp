// @author Merve Asiler

#include "Mesh.h"
#include "CommonUtils.h"

// Load the mesh in the OBJ file format
void Mesh::loadObj(const char* name)
{
	string line;
	vector<string> vertexInfoSet, cornerInfoSet;
	double vertexCoordSet[3];
	int cornerIdSet[4];

	ifstream inpFile(name), curLinePtr;
	if (inpFile.is_open()) {
		while (!inpFile.eof()) {

			getline(inpFile, line);

			// Read the vertices
			if (line[0] == 'v' && line[1] == ' ') {
				vertexInfoSet = split(line, " ");
				for (unsigned int i = 1; i < vertexInfoSet.size(); i++)
					sscanf_s(vertexInfoSet[i].c_str(), "%lf", &(vertexCoordSet[i - 1]));

				addVertex(vertexCoordSet[0], vertexCoordSet[1], vertexCoordSet[2]);
			}

			// Read the triangles
			else if (line[0] == 'f' && line[1] == ' ') {
				cornerInfoSet = split(line, " ");
				for (unsigned int i = 1; i < cornerInfoSet.size(); i++)
					sscanf_s(cornerInfoSet[i].c_str(), "%d/", &(cornerIdSet[i - 1]));

				// quad ( means 2 triangles)
				if (cornerInfoSet.size() > 4) {
					addTriangle(cornerIdSet[0] - 1, cornerIdSet[1] - 1, cornerIdSet[2] - 1);	// indices start from 1 in the file
					addTriangle(cornerIdSet[2] - 1, cornerIdSet[3] - 1, cornerIdSet[0] - 1);
				}

				// single triangle
				else
					addTriangle(cornerIdSet[0] - 1, cornerIdSet[1] - 1, cornerIdSet[2] - 1);

				cornerInfoSet.clear();
			}

			else
				continue;	// ignore "vt" and "vn" data
		}

		inpFile.close();
	}
}

// Load the mesh in the OFF file format
void Mesh::loadOff(const char* name) {

	int nVerts, nTris, dummy;
	double x, y, z;
	string line;
	int cornerId1, cornerId2, cornerId3;

	ifstream inpFile(name);
	if (inpFile.is_open()) {

		// Read OFF
		getline(inpFile, line);

		// Get the number of vertices and triangles (dummy is the number of edges)
		inpFile >> nVerts >> nTris >> dummy;

		// Read the vertices
		for (int i = 0; i < nVerts; i++) {
			inpFile >> x >> y >> z;
			addVertex(x, y, z);
		}

		// Read the triangles (dummy is the number of vertices)
		for (int i = 0; i < nTris; i++) {
			inpFile >> dummy >> cornerId1 >> cornerId2 >> cornerId3;
			addTriangle(cornerId1, cornerId2, cornerId3);
		}

		inpFile.close();
	}

}

// Load the mesh in the OFF file format
void Mesh::writeOff(string offFileName) {

	ofstream outFile;
	outFile.open(offFileName);
	outFile << "OFF" << endl;

	// Write the number of vertices and triangles (dummy is the number of edges)
	outFile << this->getNumOfVerts() << " " << this->getNumOfTris() << " 0" << endl;

	// Write the vertices
	for (int i = 0; i < this->getNumOfVerts(); i++)
		outFile << this->getVertex(i).coords[0] << " " << this->getVertex(i).coords[1] << " " <<this->getVertex(i).coords[2] << endl;

	// Write the triangles
	for (int i = 0; i < this->getNumOfTris(); i++)
		outFile << "3 " << this->getTriangle(i).corners[0] << " " << this->getTriangle(i).corners[1] << " " << this->getTriangle(i).corners[2] << endl;

	outFile.close();

}

void Mesh::addVertex(double x, double y, double z)
{
	int idx = verts.size();
	double c[3];
	c[0] = x;
	c[1] = y;
	c[2] = z;

	verts.push_back(Vertex(idx, c));
}

void Mesh::addTriangle(int v1, int v2, int v3)
{
	int idx = tris.size();
	int c[3];
	c[0] = v1;
	c[1] = v2;
	c[2] = v3;

	tris.push_back(Triangle(idx, c)); 

	//set up structure
	verts[v1].triList.push_back(idx);
	verts[v2].triList.push_back(idx);
	verts[v3].triList.push_back(idx);

	for (int i = 0; i<3; i++) {
		int corner1 = c[i];
		int corner2 = c[(i + 1) % 3];
		int edge_id = makeVertsNeighbor(corner1, corner2);
		if (edge_id < 0) {	// if the edge was not defined before
			edge_id = edges.size();
			addEdge(corner1, corner2);
		}
		else {	// construct adjacent triangle relationship
			for (int j = 0; j < edges[edge_id].triList.size(); j++) {
				Triangle& adjTriangle = tris[edges[edge_id].triList[j]];
				
				adjTriangle.triList.push_back(idx);
				tris[idx].triList.push_back(adjTriangle.idx);
			}
		}

		tris[idx].edgeList.push_back(edge_id);
		edges[edge_id].triList.push_back(idx);
	}

	tris[idx].computeNormal(&verts[v1], &verts[v2], &verts[v3]);
}

void Mesh::addEdge(int v1, int v2)
{
	int idx = edges.size();
	int c[2];
	c[0] = v1;
	c[1] = v2;
	Edge edge(idx, c);
	edges.push_back(edge);
	
	verts[v1].edgeList.push_back(idx);
	verts[v2].edgeList.push_back(idx);

}

int Mesh::makeVertsNeighbor(int v1i, int v2i)
{
	for (unsigned int i = 0; i < verts[v1i].edgeList.size(); i++) {
		Edge& e = edges[verts[v1i].edgeList[i]];
		for (int j = 0; j < 2; j++)
			if (e.endVerts[j] == v2i)
				return e.idx;
	}

	verts[v1i].vertList.push_back(v2i);
	verts[v2i].vertList.push_back(v1i);
	return -1;
}

void Mesh::addVertexColor(int id, double color[3]) {

	for (int i = 0; i < 3; i++)
		this->verts[id].color[i] = color[i];

}

void Mesh::changeVertexCoords(int id, double coords[3]) {

	for (int i = 0; i < 3; i++)
		this->verts[id].coords[i] = coords[i];

}

Mesh::~Mesh()
{
	verts.clear();
	tris.clear();
	edges.clear();
}

Mesh::Mesh(const Mesh& mesh) {

	for (int i = 0; i < mesh.getNumOfVerts(); i++) {
		Vertex v = mesh.getVertex(i);
		this->addVertex(v.coords[0], v.coords[1], v.coords[2]);
	}

	for (int i = 0; i < mesh.getNumOfTris(); i++) {
		Triangle t = mesh.getTriangle(i);
		this->addTriangle(t.corners[0], t.corners[1], t.corners[2]);
	}

}

