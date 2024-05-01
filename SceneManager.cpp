// @author Merve Asiler

#include "SceneManager.h"
#include "CommonUtils.h"

#include <string>

/* **************************************************************************** */
/************************** EVENT BASED FUNCTION CALL ***************************/

#include <Inventor/sensors/SoTimerSensor.h>		//timer
#include <Inventor/nodes/SoEventCallback.h>		//keyboard
#include <Inventor/events/SoKeyboardEvent.h>
      
struct AnimationTools {
	bool animationActive = false;
	string animationType = "drawEdgesIteratively";	// default
	int iteration = 0;
	int startTimeThreshold = 250;
	SoTimerSensor* timeSensor = nullptr;
	vector<SoSeparator*> resSet;
	vector<Mesh*> meshSet;
	Painter* painter = nullptr;
};

void keyHit(void* userData, SoEventCallback* eventCB)
{
	AnimationTools* animationTools = (AnimationTools*)userData;
	const SoEvent* theEvent = eventCB->getEvent();

	if (SO_KEY_PRESS_EVENT(theEvent, S))
	{
		cout << "started/resumed\n";
		animationTools->animationActive = true;
		animationTools->timeSensor->schedule(); //start animation
	}
	else if (SO_KEY_PRESS_EVENT(theEvent, P))
	{
		cout << "paused\n";
		animationTools->animationActive = false;
		animationTools->timeSensor->unschedule(); //stop animation (pause effect)
	}

	eventCB->setHandled();
}

void timeEvent(void* userData, SoSensor* ss)
{
	AnimationTools* animationTools = (AnimationTools*)userData;

	if (!animationTools->animationActive) {
		if (animationTools->iteration++ < animationTools->startTimeThreshold)
			return;

		animationTools->animationActive = true;
		animationTools->iteration = 0;
	}

	if (animationTools->animationActive) {
		if (animationTools->animationType == "drawEdgesIteratively")
			animationTools->painter->drawEdgesIteratively(animationTools->meshSet[0], animationTools->resSet[0], ++animationTools->iteration); //screen update within ICP by calling vertex.set1Value()s
		else {
			double angle = PI / 90.0;
			double rotationAngles[3] = { 0,angle, 0 };	// 2 degrees or not around each axis
			for (int m = 0; m < animationTools->meshSet.size(); m++) {
				double rotationCenter[3];
				if (m % 2 == 0)
					animationTools->meshSet[m]->computeCenter(rotationCenter);
				else
					animationTools->meshSet[m-1]->computeCenter(rotationCenter);
				animationTools->painter->drawRotatingMeshes(animationTools->meshSet[m], animationTools->resSet[m], rotationCenter, rotationAngles);
			}
		}
	}
	//else 
	//	animationTools->timeSensor->unschedule(); 
}
/* **************************************************************************** */
/* **************************************************************************** */

void DrawMeshToScene(string meshName) {

	Mesh* mesh = new Mesh;
	if (meshName.substr(meshName.length() - 3, 3) == "off")
		mesh->loadOff(meshName.c_str());
	else
		mesh->loadObj(meshName.c_str());

	Scene* scene = new Scene();
	Painter* painter = new Painter();
	SoSeparator* res = new SoSeparator();
	painter->getShapeSep(mesh, res);
	painter->drawTriangulation(mesh, res);
	scene->makeScene(res);
	delete scene;
	delete painter;
	delete mesh;

}

void DrawMultipleMeshToScene(vector<tuple<Mesh*, MaterialSetting*>> mesh_mat_set) {

	Scene* scene = new Scene();
	Painter* painter = new Painter();

	vector<SoSeparator*> resSet;
	for (int i = 0; i < mesh_mat_set.size(); i++) {
		Mesh* mesh;
		MaterialSetting* mat;
		tie(mesh, mat) = mesh_mat_set[i];
		SoSeparator* res = new SoSeparator();
		/*
		if (i == 0)
			painter->getShapeSepByEdges(mesh, mat, res);
		else {
		*/
			painter->getShapeSep(mesh, mat, res);
			if (i > 0)
				painter->drawTriangulation(mesh, res);
		//}
		resSet.push_back(res);
	}

	scene->makeScene(resSet);

	resSet.clear();
	delete scene;
	delete painter;

}

void DrawMultipleAnimatedMeshToScene(vector<tuple<Mesh*, MaterialSetting*>> mesh_mat_set) {

	// scene
	Scene* scene = new Scene();

	// animation tools
	AnimationTools* animationTools = new AnimationTools;
	
	// time sensor
	SoTimerSensor* timeSensor = new SoTimerSensor(timeEvent, animationTools); //default unscheduled; so not start until schedule() is called
	timeSensor->setInterval(0.01f);
	animationTools->timeSensor = timeSensor;

	// keyboard event
	SoEventCallback* eventCB = new SoEventCallback;
	eventCB->addEventCallback(SoKeyboardEvent::getClassTypeId(), keyHit, animationTools);
	scene->attachToRoot(eventCB);

	// painter
	Painter* painter = new Painter();
	animationTools->painter = painter;

	vector<SoSeparator*> resSet;
	for (int i = 0; i < mesh_mat_set.size(); i++) {
		Mesh* mesh;
		MaterialSetting* mat;
		tie(mesh, mat) = mesh_mat_set[i];
		SoSeparator* res = new SoSeparator();
		
		if (i == 1) {	// mesh
			painter->getShapeSep(mesh, mat, res);
			//painter->drawTriangulation(mesh, res);
		}
		else {		// kernel
			animationTools->meshSet.push_back(mesh);
			animationTools->resSet.push_back(res);
			//painter->drawEdgesIteratively(mesh, res, 0);
		}
	
		resSet.push_back(res);
	}

	animationTools->animationType = "drawEdgesIteratively";
	//animationTools->timeSensor->schedule();
	scene->makeScene(resSet);

	resSet.clear();
	delete scene;
	delete painter;

}

void DrawMultipleScenes(vector<tuple<tuple<Mesh*, MaterialSetting*>, tuple<Mesh*, MaterialSetting*>>> outputs, vector<double*> positions, double sceneSize) {

	Scene* scene = new Scene();
	vector<SoSeparator*> resSets;
	Painter* painter = new Painter();
	
	// SCENES
	for (int i = 0; i < outputs.size(); i++) {
		tuple<tuple<Mesh*, MaterialSetting*>, tuple<Mesh*, MaterialSetting*>> kernel_mesh_tuple = outputs[i];
		tuple<Mesh*, MaterialSetting*> kernel_mat_set, mesh_mat_set;
		tie(kernel_mat_set, mesh_mat_set) = kernel_mesh_tuple;

		Mesh* mesh[2];
		MaterialSetting* mesh_mat[2];
		tie(mesh[0], mesh_mat[0]) = kernel_mat_set;
		tie(mesh[1], mesh_mat[1]) = mesh_mat_set;

		SoSeparator* resSet = new SoSeparator;
		resSets.push_back(resSet);
		
		// compute position
		SoTransform* transform = new SoTransform();
		transform->translation.setValue(positions[i][0], positions[i][1], positions[i][2]);
		resSet->addChild(transform);
		
		for (int j = 0; j < 2; j++) {
			SoSeparator* res = new SoSeparator();
			//if (i==1 && j==0)
			//	painter->getShapeSepByEdges(mesh[j], mesh_mat[j], res);
			//else
				painter->getShapeSep(mesh[j], mesh_mat[j], res);
				if (j > 0)
					painter->drawTriangulation(mesh[j], res);
			resSet->addChild(res);
		}
	}

	scene->makeMultipleScene(resSets, sceneSize);

	// clean-up
	for (int i = 0; i < outputs.size()-1; i++)
		resSets[i]->removeAllChildren();
	resSets.clear();
	delete painter;
	delete scene;

}

void DrawMultipleAnimatedScenes(vector<tuple<tuple<Mesh*, MaterialSetting*>, tuple<Mesh*, MaterialSetting*>>> outputs, vector<double*> positions, double sceneSize) {

	// scene
	Scene* scene = new Scene();

	// animation tools
	AnimationTools* animationTools = new AnimationTools;

	// time sensor
	SoTimerSensor* timeSensor = new SoTimerSensor(timeEvent, animationTools); //default unscheduled; so not start until schedule() is called
	timeSensor->setInterval(0.01f);
	animationTools->timeSensor = timeSensor;

	// keyboard event
	SoEventCallback* eventCB = new SoEventCallback;
	eventCB->addEventCallback(SoKeyboardEvent::getClassTypeId(), keyHit, animationTools);
	scene->attachToRoot(eventCB);

	// painter
	Painter* painter = new Painter();
	animationTools->painter = painter;

	vector<SoSeparator*> resSets;

	// SCENES
	for (int i = 0; i < outputs.size(); i++) {
		tuple<tuple<Mesh*, MaterialSetting*>, tuple<Mesh*, MaterialSetting*>> kernel_mesh_tuple = outputs[i];
		tuple<Mesh*, MaterialSetting*> kernel_mat_set, mesh_mat_set;
		tie(kernel_mat_set, mesh_mat_set) = kernel_mesh_tuple;

		Mesh* mesh[2];
		MaterialSetting* mesh_mat[2];
		tie(mesh[0], mesh_mat[0]) = kernel_mat_set;
		tie(mesh[1], mesh_mat[1]) = mesh_mat_set;

		SoSeparator* resSet = new SoSeparator;
		resSets.push_back(resSet);

		// compute position
		SoTransform* transform = new SoTransform();
		transform->translation.setValue(positions[i][0], positions[i][1], positions[i][2]);
		resSet->addChild(transform);

		SoTransform* transform_r3 = new SoTransform();
		SbVec3f axis3(-1, -1, 0);
		transform_r3->rotation.setValue(axis3, PI / 3);
		//resSet->addChild(transform_r3);

		for (int j = 1; j >= 0; j--) {
			
			SoSeparator* res = new SoSeparator();
			if (j == 0)
				painter->getShapeSep(mesh[j], mesh_mat[j], res);
			else {
				if (i == 0) {
					painter->getShapeSep(mesh[j], mesh_mat[j], res);
					painter->drawTriangulation(mesh[j], res);
				}
				// if (i > 0) do not draw host mesh
			}
			
			resSet->addChild(res);

			animationTools->resSet.push_back(res);
			animationTools->meshSet.push_back(mesh[j]);
		}
	}

	animationTools->animationType = "drawRotatingMeshes";
	animationTools->timeSensor->schedule();
	scene->makeMultipleScene(resSets, sceneSize);

	// clean-up
	for (int i = 0; i < outputs.size() - 1; i++)
		resSets[i]->removeAllChildren();
	resSets.clear();
	delete painter;
	delete scene;

}
