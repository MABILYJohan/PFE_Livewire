#ifndef DIJKSTRA_H
#define DIJKSTRA_H

#include "utilsMesh.h"

#include <QVector>
#include <vector>

using namespace std;

class Dijkstra
{
public:
    Dijkstra();

    vector<int> get_paths();
    vector<int> get_currentPath();

    void dijkstra(MyMesh* _mesh, int _vertexStart);
    vector<int> calc_path(MyMesh* _mesh, int vertexEnd);

private:
    vector<int> allPaths;
    vector<int> currentPath;
    vector<int> tabWeights;
    int vertexStart;

    bool tous_sommets_extraits(QVector<bool> tab);
    int extraire_min(QVector<bool> listeSommets);
};

#endif // DIJKSTRA_H
