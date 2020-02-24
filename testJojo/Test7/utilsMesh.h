#ifndef UTILSMESH_H
#define UTILSMESH_H

#include <OpenMesh/Core/IO/MeshIO.hh>
#include <OpenMesh/Core/Mesh/TriMesh_ArrayKernelT.hh>
#include <cmath>

using namespace OpenMesh;
using namespace OpenMesh::Attributes;

#include <QVector3D>
#include <vector>

#include "utils.h"

using namespace std;

struct MyTraits : public OpenMesh::DefaultTraits
{
    // use vertex normals and vertex colors
    VertexAttributes( OpenMesh::Attributes::Normal | OpenMesh::Attributes::Color | OpenMesh::Attributes::Status);
    // store the previous halfedge
    HalfedgeAttributes( OpenMesh::Attributes::PrevHalfedge );
    // use face normals face colors
    FaceAttributes( OpenMesh::Attributes::Normal | OpenMesh::Attributes::Color | OpenMesh::Attributes::Status);
    EdgeAttributes( OpenMesh::Attributes::Color | OpenMesh::Attributes::Status );
    // vertex thickness
    VertexTraits{float thickness; float value;};
    // edge thickness
    EdgeTraits{float thickness;};
};
typedef OpenMesh::TriMesh_ArrayKernelT<MyTraits> MyMesh;


class UtilsMesh
{
private:
    static float middle_edge_coord(float p1, float p2);

    static void compter_sommets_comp_connexe(MyMesh* _mesh, int sommet,
                                             vector<bool> &vertexParcours, vector<VertexHandle> &vecCC);


public:
    UtilsMesh();


    ////////////////////////    BOOL    ///////////////////////////////////////

    static bool edge_is_in_face(MyMesh *_mesh, int edgeID, int faceID);
    static bool faces_have_common_edge(MyMesh *_mesh, int face1ID, int face2ID);
    static bool vertex_is_in_face(MyMesh *_mesh, int faceID, int vertexID);


    ////////////////////////    HANDLES    /////////////////////////////////////

    static int get_vertex_of_point(MyMesh *_mesh, MyMesh::Point p, int precision=0);
    static int get_opposite_edgeFace(MyMesh *_mesh, int faceID, int edgeID);
    static void get_vh_of_edge(MyMesh *_mesh, int edgeID,
                               VertexHandle &vh0, VertexHandle &vh1);
    static void get_eh_of_triangle_face(MyMesh *_mesh, int faceID,
            EdgeHandle &eh0, EdgeHandle &eh1, EdgeHandle &eh2);
    static vector<VertexHandle> get_vh_of_face(MyMesh *_mesh, int faceID);
    static void get_vh_of_triangle_face(MyMesh *_mesh, int faceID,
            VertexHandle &vh0, VertexHandle &vh1, VertexHandle &vh2);
    static void get_points_of_triangle_face(MyMesh *_mesh, int faceID,
            MyMesh::Point &p1, MyMesh::Point &p2, MyMesh::Point &p3);
    static vector<FaceHandle> get_neighbour_faces(MyMesh *_mesh, int numFace);
	static vector<EdgeHandle> get_edgeEdge_circulator(MyMesh *_mesh, int numEdge);
    static EdgeHandle get_next_eh_of_vh(MyMesh *_mesh, int numVertex);

    static int find_near_vertex_of_point(MyMesh *_mesh, MyMesh::Point P);


    ////////////////////////    TRANSFOS    ///////////////////////////////////

    static void add_triangle_face(MyMesh *_mesh,
                                  VertexHandle vh1,
                                  VertexHandle vh2,
                                  VertexHandle vh3);
    static void add_face(MyMesh *_mesh, vector<VertexHandle> vhs);
    static void collapseEdge(MyMesh* _mesh, int edgeID);
    static void flipEdge(MyMesh* _mesh, int edgeID);
    static void splitEdge(MyMesh* _mesh, int edgeID, bool bCopy=false);

    ////////////////////////    AUTRES    /////////////////////////////////////

    static QVector3D to_qvector3D(MyMesh::Point p);
    static float distance_point_to_face(MyMesh *_mesh, MyMesh::Point p, int faceID);
    static MyMesh::Point middle_edge(MyMesh *_mesh, int edgeID);
    static MyMesh::Point barycentre_triangle(MyMesh *_mesh, int faceID);
    static float angle_diedre(MyMesh *_mesh);

    static unsigned nb_connexity_componenents(MyMesh *_mesh);
    static void extract_biggest_connexity_component(MyMesh *_mesh);


    ////////////////////////    DOCS    /////////////////////////////////////

    // angle diedre
    /*
     * float a = _mesh->calc_dihedral_angle(EdgeHandle(eh));
     * */

    // Edge Length
    /*
     * float l = _mesh->calc_edge_length(EdgeHandle(eh)));
     * */
};

#endif // UTILS_H
