// @author Merve Asiler

#include "Scene.h"
#include <iostream>

Scene::Scene() {

	window = SoWin::init("Kernel Computation");
	viewer = new SoWinExaminerViewer(window);
	root = new SoSeparator;
	root->ref();

}

void Scene::attachToRoot(void* res) {
	root->addChild((SoNode*) res);
}

void Scene::configureViewer() {

	viewer->setBackgroundColor(SbColor(1, 1, 1));
	viewer->setSize(SbVec2s(640, 480));
	viewer->setSceneGraph(root);
	viewer->viewAll();
	viewer->show();
}

SoOrthographicCamera* Scene::configureOrthographicCamera(float cameraHeight) {

	SoOrthographicCamera* camera = new SoOrthographicCamera;
	camera->position.setValue(0, 0, 25.0);
	camera->pointAt(SbVec3f(0, 0, 1.0), SbVec3f(0, 1.0, 0));	// orientation
	camera->nearDistance.setValue(10.0);
	camera->farDistance.setValue(40.0);
	camera->focalDistance.setValue(25.0);
	camera->aspectRatio.setValue(1.0);							// SO_ASPECT_SQUARE
	camera->height = cameraHeight;

	return camera;

}

void Scene::play() {

	SoWin::show(window);
	SoWin::mainLoop();
}

Scene::~Scene() {
	root->unref();
	delete viewer;
}

void Scene::makeScene(SoSeparator* res) {

	attachToRoot(res);
	configureViewer();
	play();
}

void Scene::makeScene(vector< SoSeparator* > resSet) {

	for (auto& res : resSet)
		attachToRoot(res);
	configureViewer();
	play();
}

void Scene::makeScene(vector< SoSwitch* > resSet) {

	for (auto& res : resSet)
		attachToRoot(res);
	configureViewer();
	play();
}

void Scene::makeMultipleScene(vector<SoSeparator*> resSets, float sceneHeight) {

	SoOrthographicCamera* camera = configureOrthographicCamera(sceneHeight);

	root = new SoSeparator;
	root->addChild(camera);
	for (int i=0; i < resSets.size(); i++)
		root->addChild(resSets[i]);
	root->ref();
	configureViewer();

	play();

}



