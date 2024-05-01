// @author Merve Asiler

#pragma once

#include "Painter.h"
#include "Scene.h"
#include "Mesh.h"

// SCENE DRAWING MANAGER METHODs

void DrawMeshToScene(string meshName);

void DrawMultipleMeshToScene(vector<tuple<Mesh*, MaterialSetting*>> mesh_mat_set);

void DrawMultipleAnimatedMeshToScene(vector<tuple<Mesh*, MaterialSetting*>> mesh_mat_set);

void DrawMultipleScenes(vector<tuple<tuple<Mesh*, MaterialSetting*>, tuple<Mesh*, MaterialSetting*>>> outputs, vector<double*> positions, double sceneSize);

void DrawMultipleAnimatedScenes(vector<tuple<tuple<Mesh*, MaterialSetting*>, tuple<Mesh*, MaterialSetting*>>> outputs, vector<double*> positions, double sceneSize);

