// @author Merve Asiler

#include "CGALUtils.h"

Mesh convertCGALMeshToMesh(CGALMesh sm) {

    Mesh mesh;
    
    for (CGALMesh::Vertex_index vi : sm.vertices()) {
        CGALPoint pt = sm.point(vi);
        mesh.addVertex(CGAL::to_double(pt.x()), CGAL::to_double(pt.y()), CGAL::to_double(pt.z()));
    }

    for (CGALMesh::Face_index face_index : sm.faces()) {
        vector<uint32_t> indices;
        for (CGALMesh::Vertex_index vi : vertices_around_face(sm.halfedge(face_index), sm))
            indices.push_back(vi.idx());
        while (indices.size() >= 3) {
            mesh.addTriangle(indices[0], indices[1], indices[2]);
            indices.erase(indices.begin() + 1);
        }
    }

    return mesh;
}

Mesh convertCGALPolyhedronToMesh(CGALPolyhedron poly) {

    Mesh mesh;

    // save vertices
    for (Vertex_iterator i = poly.points_begin(); i != poly.points_end(); ++i)
        mesh.addVertex(CGAL::to_double(i->x()), CGAL::to_double(i->y()), CGAL::to_double(i->z()));

    // save triangles
    for (Facet_iterator i = poly.facets_begin(); i != poly.facets_end(); ++i) {
        Halfedge_facet_circulator j = i->facet_begin();
        vector<int> vertex_indices;
        do {
            vertex_indices.push_back(std::distance(poly.vertices_begin(), j->vertex()));
        } while (++j != i->facet_begin());
        
        while (vertex_indices.size() >= 3) {
            mesh.addTriangle(vertex_indices[0], vertex_indices[1], vertex_indices[2]);
            vertex_indices.erase(vertex_indices.begin() + 1);
        }
    }

    return mesh;

}

CGALMesh convertMeshToCGALMesh(Mesh& mesh) {

    CGALMesh surface_mesh;
    vector<CGALMesh::Vertex_index> vi;

    // Add the points as vertices
    for (int i = 0; i < mesh.getNumOfVerts(); i++) {
        Vertex v = mesh.getVertex(i);
        vi.push_back(surface_mesh.add_vertex(CGALPoint(v.coords[0], v.coords[1], v.coords[2])));
    }

    // Add the triangles as faces
    for (int i = 0; i < mesh.getNumOfTris(); i++) {
        Triangle t = mesh.getTriangle(i);
        CGALMesh::Face_index fi = surface_mesh.add_face(vi[t.corners[0]], vi[t.corners[1]], vi[t.corners[2]]);
    }

    vi.clear();
    return surface_mesh;

}

CGALMesh convertMeshToCGALGraphMesh(Mesh& mesh) {

    CGALMesh surface_mesh;
    vector<CGALPoint> points;

    // Add the points as vertices
    for (int i = 0; i < mesh.getNumOfVerts(); i++) {
        Vertex v = mesh.getVertex(i);
        points.push_back(CGALPoint(v.coords[0], v.coords[1], v.coords[2]));
    }

    // Add the triangles as faces
    for (int i = 0; i < mesh.getNumOfTris(); i++) {
        Triangle t = mesh.getTriangle(i);
        CGAL::make_triangle(points[t.corners[0]], points[t.corners[1]], points[t.corners[2]], surface_mesh);
    }

    points.clear();
    return surface_mesh;

}

double* computeHausdorffDistance(Mesh& mesh1, Mesh& mesh2) {

    CGALMesh sm1 = convertMeshToCGALMesh(mesh1);
    CGALMesh sm2 = convertMeshToCGALMesh(mesh2);

    double* hd = new double[3];
    hd[0] = CGAL::Polygon_mesh_processing::approximate_Hausdorff_distance<TAG>(sm1, sm2);
    hd[1] = CGAL::Polygon_mesh_processing::approximate_Hausdorff_distance<TAG>(sm2, sm1);
    hd[2] = CGAL::Polygon_mesh_processing::approximate_symmetric_Hausdorff_distance <TAG>(sm1, sm2);
    return hd;

}

Mesh computeConvexHull(const vector<Vertex>& vertices) {

    // convert vertices to CGALPoint array
    std::vector<CGALPoint> points;
    for (int i = 0; i < vertices.size(); i++) {
        Vertex v = vertices[i];
        CGALPoint p(v.coords[0], v.coords[1], v.coords[2]);
        points.push_back(p);
    }

    // define polyhedron to hold convex hull
    CGALPolyhedron chull;
    // compute convex hull of non-collinear points
    CGAL::convex_hull_3(points.begin(), points.end(), chull);

    return convertCGALPolyhedronToMesh(chull);
    //return convertCGALPolyhedronToMesh_ForVideo(chull);
}

Mesh computeConvexHull(string meshName) {
    Mesh inputMesh;
    if (meshName.substr(meshName.length() - 3, 3) == "off")
        inputMesh.loadOff(meshName.c_str());
    else
        inputMesh.loadObj(meshName.c_str());

    Mesh mesh = computeConvexHull(inputMesh.getAllVerts());
    return mesh;

}

Mesh convertCGALPolyhedronToMesh_ForVideo(CGALPolyhedron poly) {

    Mesh mesh;

    // save vertices
    for (Vertex_iterator i = poly.points_begin(); i != poly.points_end(); ++i)
        mesh.addVertex(CGAL::to_double(i->x()), CGAL::to_double(i->y()), CGAL::to_double(i->z()));

    // save triangles
    for (Facet_iterator i = poly.facets_begin(); i != poly.facets_end(); ++i) {
        Halfedge_facet_circulator j = i->facet_begin();
        vector<int> vertex_indices;
        do {
            vertex_indices.push_back(std::distance(poly.vertices_begin(), j->vertex()));
        } while (++j != i->facet_begin());

        for (int v = 0; v < vertex_indices.size(); v++)
            mesh.addEdge(vertex_indices[v], vertex_indices[(v + 1) % vertex_indices.size()]);
    }

    //cout << mesh.getNumOfEdges() << endl;
    Mesh kernel;
    vector<int> vertex_id_list;
    vector<int> edge_id_list;
    queue<int> to_be_traced_edge_id_list;
    int v1 = 0;
    Edge edge = mesh.getEdge(mesh.getVertex(v1).edgeList[0]);
    int v2;
    if (edge.endVerts[0] == v1)
        v2 = edge.endVerts[1];
    else
        v2 = edge.endVerts[0];

    kernel.addVertex(mesh.getVertex(v1).coords[0], mesh.getVertex(v1).coords[1], mesh.getVertex(v1).coords[2]);
    kernel.addVertex(mesh.getVertex(v2).coords[0], mesh.getVertex(v2).coords[1], mesh.getVertex(v2).coords[2]);
    vertex_id_list.push_back(v1);
    vertex_id_list.push_back(v2);
    edge_id_list.push_back(mesh.getVertex(v1).edgeList[0]);
    kernel.addEdge(0, 1);

    for (int e = 1; e < mesh.getVertex(v1).edgeList.size(); e++) {
        to_be_traced_edge_id_list.push(mesh.getVertex(v1).edgeList[e]);
        edge_id_list.push_back(mesh.getVertex(v1).edgeList[e]);
    }

    while (!to_be_traced_edge_id_list.empty()) {

        int edge_id = to_be_traced_edge_id_list.front();
        to_be_traced_edge_id_list.pop();

        edge = mesh.getEdge(edge_id);
        v1 = edge.endVerts[0];
        v2 = edge.endVerts[1];

        int v1_id = -1;
        bool v1_exists = false;
        for (int v = 0; v < vertex_id_list.size(); v++) {
            if (vertex_id_list[v] == v1) {
                v1_exists = true;
                v1_id = v;
                break;
            }
        }

        int v2_id = -1;
        bool v2_exists = false;
        for (int v = 0; v < vertex_id_list.size(); v++) {
            if (vertex_id_list[v] == v2) {
                v2_exists = true;
                v2_id = v;
                break;
            }
        }

        if (v1_exists && !v2_exists) {
            v2_id = kernel.getNumOfVerts();
            kernel.addVertex(mesh.getVertex(v2).coords[0], mesh.getVertex(v2).coords[1], mesh.getVertex(v2).coords[2]);
            vertex_id_list.push_back(v2);
            for (int e = 0; e < mesh.getVertex(v2).edgeList.size(); e++) {
                int new_edge_id = mesh.getVertex(v2).edgeList[e];
                bool previous_edge = false;
                for (int x = 0; x < edge_id_list.size(); x++)
                    if (edge_id_list[x] == new_edge_id) {
                        previous_edge = true;
                        break;
                    }
                if (!previous_edge) {
                    to_be_traced_edge_id_list.push(new_edge_id);
                    edge_id_list.push_back(new_edge_id);
                }
            }
        }
        else if (v2_exists && !v1_exists) {
            v1_id = kernel.getNumOfVerts();
            kernel.addVertex(mesh.getVertex(v1).coords[0], mesh.getVertex(v1).coords[1], mesh.getVertex(v1).coords[2]);
            vertex_id_list.push_back(v1);
            for (int e = 0; e < mesh.getVertex(v1).edgeList.size(); e++) {
                int new_edge_id = mesh.getVertex(v1).edgeList[e];
                bool previous_edge = false;
                for (int x = 0; x < edge_id_list.size(); x++)
                    if (edge_id_list[x] == new_edge_id) {
                        previous_edge = true;
                        break;
                    }
                if (!previous_edge) {
                    to_be_traced_edge_id_list.push(new_edge_id);
                    edge_id_list.push_back(new_edge_id);
                }
            }
        }

        kernel.addEdge(v1_id, v2_id);

    }
    //cout << kernel.getNumOfEdges() << endl;
    return kernel;
}


