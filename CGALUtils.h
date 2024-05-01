// @author Merve Asiler

#pragma once

#include "Mesh.h"
#include <CGAL/Exact_predicates_inexact_constructions_kernel.h>
#include <CGAL/Simple_cartesian.h>
#include <CGAL/Surface_mesh.h>
#include <CGAL/Polyhedron_3.h>
#include <CGAL/Polygon_mesh_processing/distance.h>
#include <CGAL/Polygon_mesh_processing/remesh.h>
#include <CGAL/convex_hull_3.h>
#include <CGAL/Kernel_traits.h>

#define TAG CGAL::Parallel_if_available_tag

//typedef CGAL::Exact_predicates_exact_constructions_kernel		K;
typedef CGAL::Exact_predicates_inexact_constructions_kernel		K;
//typedef CGAL::Simple_cartesian<CGAL::Interval_nt_advanced>	K;
//typedef CGAL::Simple_cartesian<double>						K;
typedef K::Plane_3												CGALPlane;
typedef K::Point_3												CGALPoint;
typedef K::Segment_3											Segment_3;
typedef K::Triangle_3											Triangle_3;
typedef CGAL::Surface_mesh<CGALPoint>							CGALMesh;
typedef CGAL::Polyhedron_3<K>									CGALPolyhedron;
typedef CGALPolyhedron::Facet_iterator							Facet_iterator;
typedef CGALPolyhedron::Point_iterator							Vertex_iterator;
typedef CGALPolyhedron::Halfedge_around_facet_circulator		Halfedge_facet_circulator;

namespace PMP = CGAL::Polygon_mesh_processing;

Mesh convertCGALMeshToMesh(CGALMesh sm);
Mesh convertCGALPolyhedronToMesh(CGALPolyhedron poly);
CGALMesh convertMeshToCGALMesh(Mesh& mesh);
CGALMesh convertMeshToCGALGraphMesh(Mesh& mesh);
double* computeHausdorffDistance(Mesh& mesh1, Mesh& mesh2);
Mesh computeConvexHull(const vector<Vertex>& vertices);
Mesh computeConvexHull(string meshName);
Mesh convertCGALPolyhedronToMesh_ForVideo(CGALPolyhedron poly);
