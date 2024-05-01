// @author Merve Asiler

#pragma once

#include <iostream>
#include <fstream>
#include <sstream>
#include <string>
#include <stdio.h>
#include "BasicMeshElements.h"

class Mesh
{

private:
	
	vector< Vertex > verts;
	vector< Triangle > tris;
	vector< Edge > edges;

public:
	Mesh() {};
	~Mesh();
	Mesh(const Mesh& mesh);
	void loadObj(const char* name);
	void loadOff(const char* name);
	void writeOff(string offFileName);

	// methods to construct mesh
	void addTriangle(int v1, int v2, int v3);
	void addEdge(int v1, int v2);
	void addVertex(double x, double y, double z);
	int makeVertsNeighbor(int v1i, int v2i);

	// methods to reach mesh elements
	int getNumOfVerts() const { return verts.size(); };
	int getNumOfTris() const { return tris.size(); };
	int getNumOfEdges() const { return edges.size(); };
	Vertex getVertex(int i) const { return verts[i]; };
	Triangle getTriangle(int i) const { return tris[i]; };
	Edge getEdge(int i) const { return edges[i]; };
	const vector<Vertex>& getAllVerts() const { return verts; };
	const vector<Triangle>& getAllTris() const { return tris; };
	const vector<Edge>& getAllEdges() const { return edges; };

	// methods to compute mesh features
	bool isManifold();
	void addVertexColor(int id, double color[3]);
	void changeVertexCoords(int id, double coords[3]);
	void computeTrisAngles();
	double computeVolume();
	void tetrahedralizeSurface();
	vector<double> computeCurvaturePerVertex();
	void makeConvex();
	void removeAbnormalTris();
	void rotate(double angles[3], double rotationCenter[3]);
	void translate(double distances[3]);
	void computeCenter(double center[3]);
};

#pragma once