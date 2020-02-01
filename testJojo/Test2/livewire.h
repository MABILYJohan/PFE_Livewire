#ifndef LIVEWIRE_H
#define LIVEWIRE_H

#include <OpenMesh/Core/IO/MeshIO.hh>
#include <OpenMesh/Core/Mesh/TriMesh_ArrayKernelT.hh>

#include "utils.h"
#include "utilsMesh.h"
#include "contour.h"

#include <list>

using namespace std;

using namespace OpenMesh;
using namespace OpenMesh::Attributes;

class LiveWire
{
public:
    LiveWire(MyMesh &_mesh, Contour _myContour);

    // Exemple
    void draw();

    void build();
    vector<unsigned> get_paths();

private:
    MyMesh &mesh;
    Contour myContour;
    vector<unsigned> paths;

    double cost_function(int numEdgeCur, int numEdgeNeigh);
    void draw_part(unsigned edge1, unsigned edge2);
};

#endif // LIVEWIRE_H
