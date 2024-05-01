// @author Merve Asiler

#include "Painter.h"
#include "BaseGeoOpUtils.h"

void Painter::getShapeSep(Mesh* mesh, SoSeparator* res)
{
	// Paint all vertices with the same color
	SoMaterial* mat = new SoMaterial();
	mat->diffuseColor.setValue(1, 1, 1);
	mat->transparency = 0.5;
	res->addChild(mat);

	// Gouraud shading
	SoShapeHints* hints = new SoShapeHints;
	hints->creaseAngle = 3.14;
	res->addChild(hints);

	SoCoordinate3* coords = new SoCoordinate3();
	for (int c = 0; c < mesh->getNumOfVerts(); c++)
		coords->point.set1Value(c, mesh->getVertex(c).coords[0],
			mesh->getVertex(c).coords[1],
			mesh->getVertex(c).coords[2]);

	SoIndexedFaceSet* faceSet = new SoIndexedFaceSet();
	for (int c = 0; c < mesh->getNumOfTris(); c++)
	{
		faceSet->coordIndex.set1Value(c * 4, mesh->getTriangle(c).corners[0]);
		faceSet->coordIndex.set1Value(c * 4 + 1, mesh->getTriangle(c).corners[1]);
		faceSet->coordIndex.set1Value(c * 4 + 2, mesh->getTriangle(c).corners[2]);
		faceSet->coordIndex.set1Value(c * 4 + 3, -1);
	}

	res->addChild(coords);
	res->addChild(faceSet);
}

void Painter::getShapeSep(Mesh* mesh, MaterialSetting* materialSetting, SoSeparator* res)
{
	// Paint all vertices with the same color
	SoMaterial* mat = new SoMaterial();
	mat->diffuseColor.setValue(materialSetting->getRed(), materialSetting->getGreen(), materialSetting->getBlue());		// 0.2, 0.3, 0.5
	mat->transparency = materialSetting->getTransparency();																// 0.7
	res->addChild(mat);

	// Gouraud shading
	SoShapeHints* hints = new SoShapeHints;
	hints->creaseAngle = 3.14;
	res->addChild(hints);

	SoCoordinate3* coords = new SoCoordinate3();
	for (int c = 0; c < mesh->getNumOfVerts(); c++)
		coords->point.set1Value(c, mesh->getVertex(c).coords[0],
			mesh->getVertex(c).coords[1],
			mesh->getVertex(c).coords[2]);

	SoIndexedFaceSet* faceSet = new SoIndexedFaceSet();
	for (int c = 0; c < mesh->getNumOfTris(); c++)
	{
		faceSet->coordIndex.set1Value(c * 4, mesh->getTriangle(c).corners[0]);
		faceSet->coordIndex.set1Value(c * 4 + 1, mesh->getTriangle(c).corners[1]);
		faceSet->coordIndex.set1Value(c * 4 + 2, mesh->getTriangle(c).corners[2]);
		faceSet->coordIndex.set1Value(c * 4 + 3, -1);		
	}

	res->addChild(coords);
	res->addChild(faceSet);
}

void Painter::getShapeSepByEdges(Mesh* mesh, MaterialSetting* materialSetting, SoSeparator* res)
{
	// Paint all vertices with the same color
	SoMaterial* mat = new SoMaterial();
	mat->diffuseColor.setValue(materialSetting->getRed(), materialSetting->getGreen(), materialSetting->getBlue());		// 0.2, 0.3, 0.5
	mat->transparency = materialSetting->getTransparency();																// 0.7
	res->addChild(mat);

	// Gouraud shading
	SoShapeHints* hints = new SoShapeHints;
	hints->creaseAngle = 3.14;
	res->addChild(hints);

	SoCoordinate3* coords = new SoCoordinate3();
	for (int c = 0; c < mesh->getNumOfVerts(); c++)
		coords->point.set1Value(c, mesh->getVertex(c).coords[0],
			mesh->getVertex(c).coords[1],
			mesh->getVertex(c).coords[2]);

	SoDrawStyle* sty = new SoDrawStyle;
	sty->lineWidth = 1.0f;
	res->addChild(sty);
	SoIndexedLineSet* ilsSet = new SoIndexedLineSet;
	for (int c = 0; c < mesh->getNumOfEdges(); c++)
	{
		ilsSet->coordIndex.set1Value(c * 2, mesh->getEdge(c).endVerts[0]);
		ilsSet->coordIndex.set1Value(c * 2 + 1, mesh->getEdge(c).endVerts[1]);
		ilsSet->coordIndex.set1Value(2, -1);
	}

	res->addChild(coords);
	res->addChild(ilsSet);
}

void Painter::drawEdgesIteratively(Mesh* mesh, SoSeparator* res, int iteration) {

	if (iteration > mesh->getNumOfEdges())
		return;

	SoMaterial* ma = new SoMaterial;
	ma->diffuseColor.set1Value(0, 0, 0, 1.0);

	SoSeparator* thickEdgeSep = new SoSeparator;
	thickEdgeSep->addChild(ma);

	SoDrawStyle* sty = new SoDrawStyle;
	sty->lineWidth = 2.0f;
	thickEdgeSep->addChild(sty);

	SoIndexedLineSet* ils = new SoIndexedLineSet;
	SoCoordinate3* co = new SoCoordinate3;
	for (int se = 0; se < iteration; se++)
	{
		co->point.set1Value(2 * se, mesh->getVertex(mesh->getEdge(se).endVerts[0]).coords[0],
			mesh->getVertex(mesh->getEdge(se).endVerts[0]).coords[1],
			mesh->getVertex(mesh->getEdge(se).endVerts[0]).coords[2]);
		co->point.set1Value(2 * se + 1, mesh->getVertex(mesh->getEdge(se).endVerts[1]).coords[0],
			mesh->getVertex(mesh->getEdge(se).endVerts[1]).coords[1],
			mesh->getVertex(mesh->getEdge(se).endVerts[1]).coords[2]);
	}

	for (int ci = 0; ci < iteration; ci++)
	{
		ils->coordIndex.set1Value(3 * ci, 2 * ci);
		ils->coordIndex.set1Value(3 * ci + 1, 2 * ci + 1);
		ils->coordIndex.set1Value(3 * ci + 2, -1);
	}

	thickEdgeSep->addChild(co);
	thickEdgeSep->addChild(ils);

	if (res->getNumChildren() == 0)
		res->addChild(thickEdgeSep);
	else
		res->replaceChild(0, thickEdgeSep);

}

void Painter::drawRotatingMeshes(Mesh* mesh, SoSeparator* res, double rotationCenter[3], double rotationAngles[3]) {

	if (res->getNumChildren() == 0)
		return;

	mesh->rotate(rotationAngles, rotationCenter);

	for (int v = 0; v < mesh->getNumOfVerts(); v++) {
		((SoCoordinate3*)res->getChild(2))->point.set1Value(v, mesh->getVertex(v).coords[0],
			mesh->getVertex(v).coords[1],
			mesh->getVertex(v).coords[2]);
	}
	/*
	if (res->getNumChildren() == 5) {

		for (int e = 0; e < mesh->getNumOfEdges(); e++) {

			((SoCoordinate3*)((SoSeparator*)res->getChild(4))->getChild(2))->point.set1Value(2 * e,
				mesh->getVertex(mesh->getEdge(e).endVerts[0]).coords[0],
				mesh->getVertex(mesh->getEdge(e).endVerts[0]).coords[1],
				mesh->getVertex(mesh->getEdge(e).endVerts[0]).coords[2]);

			((SoCoordinate3*)((SoSeparator*)res->getChild(4))->getChild(2))->point.set1Value(2 * e + 1,
				mesh->getVertex(mesh->getEdge(e).endVerts[1]).coords[0],
				mesh->getVertex(mesh->getEdge(e).endVerts[1]).coords[1],
				mesh->getVertex(mesh->getEdge(e).endVerts[1]).coords[2]);
		}

	}
	*/
}

void Painter::drawTriangulation(Mesh* mesh, SoSeparator* res) {

	SoSeparator* thickEdgeSep = new SoSeparator;
	SoMaterial* ma = new SoMaterial;
	//ma->diffuseColor.set1Value(1.0, 1.0, 1.0, 0.5);
	ma->diffuseColor.set1Value(0, 1.0, 0, 0);
	ma->transparency = 0.0;
	thickEdgeSep->addChild(ma);
	SoDrawStyle* sty = new SoDrawStyle;
	sty->lineWidth = 1.0f;
	thickEdgeSep->addChild(sty);

	SoIndexedLineSet* ils = new SoIndexedLineSet;
	SoCoordinate3* co = new SoCoordinate3;
	for (int se = 0; se < mesh->getNumOfEdges(); se++)
	{
		co->point.set1Value(2 * se, mesh->getVertex(mesh->getEdge(se).endVerts[0]).coords[0],
									mesh->getVertex(mesh->getEdge(se).endVerts[0]).coords[1], 
									mesh->getVertex(mesh->getEdge(se).endVerts[0]).coords[2]);
		co->point.set1Value(2 * se + 1, mesh->getVertex(mesh->getEdge(se).endVerts[1]).coords[0],
										mesh->getVertex(mesh->getEdge(se).endVerts[1]).coords[1], 
										mesh->getVertex(mesh->getEdge(se).endVerts[1]).coords[2]);
	}

	for (int ci = 0; ci < mesh->getNumOfEdges(); ci++)
	{
		ils->coordIndex.set1Value(3 * ci, 2 * ci);
		ils->coordIndex.set1Value(3 * ci + 1, 2 * ci + 1);
		ils->coordIndex.set1Value(3 * ci + 2, -1);
	}

	thickEdgeSep->addChild(co);
	thickEdgeSep->addChild(ils);
	res->addChild(thickEdgeSep);

}




