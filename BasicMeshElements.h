// @author Merve Asiler

#pragma once

#include "BaseMathOpUtils.h"

struct Vertex
{
	int idx;
	double coords[3];
	double color[3];	//rgb

	// adjacencies
	vector< int > vertList;
	vector< int > triList;
	vector< int > edgeList;

	Vertex() {};
	Vertex(int i, double* c);
	~Vertex();
};

struct Edge
{
	int idx;
	int endVerts[2];
	double length;

	// adjacency
	vector< int > triList;

	Edge(int i, int* c);
	~Edge();
	void computeLength(Vertex* v1, Vertex* v2);
};

struct Triangle
{
	int idx;
	int corners[3];
	double normal[3];
	vector< int > edgeList;
	vector< int > triList; // neighbor tris
	vector< double > angleList;
	double angularSkewness;
	double squish;

	Triangle() {};
	Triangle(int i, int* c);
	Triangle(const Triangle& t);	// shallow copy
	~Triangle();

	void computeNormal(Vertex* v1, Vertex* v2, Vertex* v3);

	double* computeCenter(Vertex* v1, Vertex* v2, Vertex* v3);

	double* computeAreaVector(Vertex* v1, Vertex* v2, Vertex* v3);

	void setAngSkewness(double angSkewness);

	double getAngSkewness();

	void setSquish(double squish);

	double getSquish();

};

struct TriangleWithVerts : public Triangle {
	Vertex* vertices[3];
	TriangleWithVerts(const Triangle& t, Vertex* v1, Vertex* v2, Vertex* v3) : Triangle(t) { 
		vertices[0] = v1; 
		vertices[1] = v2;
		vertices[2] = v3;
	}
	~TriangleWithVerts();
	double* computeBarycentricCoords(double point[3]);
};

