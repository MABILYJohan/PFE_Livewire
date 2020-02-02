#ifndef LIVEWIRE_H
#define LIVEWIRE_H

#include <OpenMesh/Core/IO/MeshIO.hh>
#include <OpenMesh/Core/Mesh/TriMesh_ArrayKernelT.hh>

#include "utils.h"
#include "utilsMesh.h"

#include <list>

using namespace std;

using namespace OpenMesh;
using namespace OpenMesh::Attributes;

class LiveWire
{
public:
    LiveWire(MyMesh &_mesh, int _edgeSeed, MyMesh::Point _sightPoint);

    void draw(unsigned edge2);

    void build();
    vector<int> get_paths();

private:
    MyMesh &mesh;
    vector<int> paths;
    int edgeSeed;
    MyMesh::Point sightPoint;

    double normal_orientation(int numEdge, MyMesh::Point _sightPoint);

    double cost_function(int numEdgeCur, int numEdgeNeigh);
    //    void draw_part(unsigned edge1, unsigned edge2);
};

#endif // LIVEWIRE_H
