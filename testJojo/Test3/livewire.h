#ifndef LIVEWIRE_H
#define LIVEWIRE_H

#include <OpenMesh/Core/IO/MeshIO.hh>
#include <OpenMesh/Core/Mesh/TriMesh_ArrayKernelT.hh>

#include "utils.h"
#include "utilsMesh.h"
#include "dijkstra.h"

#include <list>

using namespace std;

using namespace OpenMesh;
using namespace OpenMesh::Attributes;

class LiveWire
{
public:
    LiveWire(MyMesh &_mesh, int _vertexSeed, MyMesh::Point _sightPoint);

    void update_vertexSeed(int _vertexSeed);

    void draw(unsigned vertex2);

    void build_paths(int _vertexSeed);
    vector<int> get_paths();

private:
    MyMesh &mesh;
    vector<int> paths;
    Dijkstra myDijkstra;
    int vertexSeed;
    int edgeSeed;
    MyMesh::Point sightPoint;
    bool flag_criterion_visibility;

    vector<vector<double>> tabCosts;
    void init_criterions(unsigned nbCriterions);

    double criterion_length(EdgeHandle eh);
    double criterion_diedral_angle(EdgeHandle eh);
    double criterion_normal_orientation(EdgeHandle eh, MyMesh::Point _sightPoint);
    double criterion_visibility(EdgeHandle eh);

    double cost_function(int numEdgeCur, int numEdgeNeigh);
    //    void draw_part(unsigned edge1, unsigned edge2);
};

#endif // LIVEWIRE_H
