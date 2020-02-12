#ifndef LIVEWIRE_H
#define LIVEWIRE_H

#include <OpenMesh/Core/IO/MeshIO.hh>
#include <OpenMesh/Core/Mesh/TriMesh_ArrayKernelT.hh>

#include "utils.h"
#include "utilsMesh.h"
#include "dijkstra.h"

#include <math.h>

using namespace std;

using namespace OpenMesh;
using namespace OpenMesh::Attributes;

enum enum_criteres {LENGTH, DIEDRAL, NORMAL_OR, VISIBILITY, CURVATURE};

class LiveWire
{
public:
    LiveWire(MyMesh &_mesh, int _vertexSeed, MyMesh::Point _sightPoint);

    void display_criterions(int profDisplay = 0);

    void update_vertexSeed(int _vertexSeed, int vertexNext);

    void draw(unsigned vertex2);

    void build_paths(int vertexNext);
    vector<int> get_paths();

private:
    MyMesh &mesh;
    vector<int> paths;
    Dijkstra myDijkstra;
    int vertexSeed;
    int edgeSeed;
    double minCurv;
    double maxCurv;
    MyMesh::Point sightPoint;

    int nbMaxCrit = 5;
    vector<int> criteres;   // un tableau d'enum
    // Tableau de tableaux de coûts précalculés (en fct de critères)
    vector<vector<double>> tabCosts;

    void init_criterions();
    double criterion_length(EdgeHandle eh);
    double criterion_diedral_angle(EdgeHandle eh);
    double criterion_normal_orientation(EdgeHandle eh, MyMesh::Point _sightPoint);
    void init_min_max_curvature(MyMesh mesh);
    double criterion_curvature(EdgeHandle eh);
    double criterion_visibility(EdgeHandle eh);

    //fonctions pour la courbure gaussienne
    float angleEE(MyMesh* _mesh, int vertexID,  int faceID);
    float faceArea(MyMesh* _mesh, int faceID);
    float aire_barycentrique(MyMesh* _mesh, int vertID);
    double K_Curv(MyMesh* _mesh, int vertID);

    double cost_function(int numEdgeCur, int numEdgeNeigh);
};

#endif // LIVEWIRE_H
