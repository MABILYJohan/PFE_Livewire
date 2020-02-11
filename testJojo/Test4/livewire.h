#ifndef LIVEWIRE_H
#define LIVEWIRE_H

#include <OpenMesh/Core/IO/MeshIO.hh>
#include <OpenMesh/Core/Mesh/TriMesh_ArrayKernelT.hh>

#include "utils.h"
#include "utilsMesh.h"
#include "dijkstra.h"


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
    MyMesh::Point sightPoint;

    int nbMaxCrit = 5;
    vector<int> criteres;   // un tableau d'enum
    // Tableau de tableaux de coûts précalculés (en fct de critères)
    vector<vector<double>> tabCosts;

    void init_criterions();
    double criterion_length(EdgeHandle eh);
    double criterion_diedral_angle(EdgeHandle eh);
    double criterion_normal_orientation(EdgeHandle eh, MyMesh::Point _sightPoint);
    double criterion_visibility(EdgeHandle eh);

    double cost_function(int numEdgeCur, int numEdgeNeigh);
};

#endif // LIVEWIRE_H
