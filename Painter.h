// @author Merve Asiler

#include <Inventor/nodes/SoMaterial.h>
#include <Inventor/nodes/SoCoordinate3.h>
#include <Inventor/nodes/SoIndexedFaceSet.h>
#include <Inventor/nodes/SoSeparator.h>
#include <Inventor/nodes/SoShapeHints.h>
#include <Inventor/nodes/SoLineSet.h>
#include <Inventor/nodes/SoDrawStyle.h>
#include <Inventor/nodes/SoIndexedLineSet.h>
#include <Inventor/nodes/SoTransform.h>
#include <Inventor/nodes/SoSphere.h>
#include <Inventor/actions/SoAction.h>
#include <Inventor/nodes/SoAsciiText.h>

#include "Mesh.h"

class MaterialSetting {
	float red, green, blue;
	float transparency;

public:
	MaterialSetting(float r, float g, float b, float t) : red(r), green(g), blue(b), transparency(t) {};
	float getRed() { return red; }
	float getGreen() { return green; }
	float getBlue() { return blue; }
	float getTransparency() { return transparency; }
	void setTransparency(float t) { transparency = t; };
	void setRGB(float r, float g, float b) { red = r; green = g; blue = b; };
};

class Painter
{
public:
	void getShapeSep(Mesh* mesh, SoSeparator* res);
	void getShapeSep(Mesh* mesh, MaterialSetting* materialSetting, SoSeparator* res);
	void getShapeSepByEdges(Mesh* mesh, MaterialSetting* materialSetting, SoSeparator* res);
	void drawEdgesIteratively(Mesh* mesh, SoSeparator* res, int iteration);
	void drawRotatingMeshes(Mesh* mesh, SoSeparator* res, double rotationCenter[3], double rotationAngles[3]);
	void drawTriangulation(Mesh* mesh, SoSeparator* res);
};

#pragma once