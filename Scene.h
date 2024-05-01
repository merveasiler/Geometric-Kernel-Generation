// @author Merve Asiler

#pragma once

#include <Inventor/Win/SoWin.h>
#include <Inventor/Win/SoWinRenderArea.h>
#include <Inventor/Win/viewers/SoWinExaminerViewer.h>
#include <Inventor/nodes/SoCube.h>
#include <Inventor/nodes/SoSeparator.h>
#include <Inventor/nodes/SoSelection.h>
#include <Inventor/nodes/SoCamera.h>
#include <Inventor/nodes/SoOrthographicCamera.h>
#include <Inventor/nodes/SoPerspectiveCamera.h>
#include <Inventor/nodes/SoSwitch.h>
#include <Inventor/nodes/SoTransform.h>
#include <Inventor/SoPickedPoint.h>
#include <vector>

using namespace std;

SoPath* pickFilterCB(void*, const SoPickedPoint* pick);

class Scene {

	HWND window;
	SoWinExaminerViewer * viewer;
	//SoSeparator * root;
	SoSelection * selection;

	vector<HWND*> windows;
	vector<SoWinExaminerViewer*> viewers;
	vector<SoSeparator*> roots;
	int numOfScenes;
	float cameraHeight;


public:
	SoSeparator* root;

	Scene();

	void attachToRoot(void* res);

	void configureViewer();

	SoOrthographicCamera* configureOrthographicCamera(float cameraHeight);

	void play();

	~Scene();

	void makeScene(SoSeparator* res);

	void makeScene(vector< SoSeparator* > resSet);

	void makeScene(vector< SoSwitch* > resSet);

	void makeMultipleScene(vector< SoSeparator* > resSets, float sceneHeight);

};
