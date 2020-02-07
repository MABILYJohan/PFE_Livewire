#include "dijkstra.h"

#include <QDebug>

Dijkstra::Dijkstra()
{

}

vector<int> Dijkstra::get_paths()   {   return  allPaths;  }
vector<int> Dijkstra::get_currentPath() {   return currentPath; }

bool Dijkstra::tous_sommets_extraits(QVector<bool> tab)
{
    for(int i=0; i<tab.size(); i++){
        if(tab[i]){
            return false;
        }
    }
    return true;
}

int Dijkstra::extraire_min(QVector<bool> listeSommets)
{
    int poid_min = INT32_MAX, sommet_min;
    for(int i=0; i<listeSommets.size(); i++){
        if(listeSommets[i] && (tabWeights[i] <= poid_min)){
            poid_min = tabWeights[i];
            sommet_min = i;
        }
    }
    return sommet_min;
}

void Dijkstra::dijkstra(MyMesh* _mesh, int _vertexStart)
{
    vertexStart = _vertexStart;
    int nbVertex = _mesh->n_vertices();
    QVector<bool> vertexList;
    allPaths.clear();
    currentPath.clear();
    tabWeights.clear();

    // INIT
    for(int v=0; v<nbVertex; v++){
        //On initialise nos valeurs pour utiliser dijkstra
        vertexList.append(true);
        tabWeights.push_back(INT32_MAX);
        allPaths.push_back(-1);
    }
    tabWeights[vertexStart] = 0;

    while(!tous_sommets_extraits(vertexList))
    {
        int vertex = extraire_min(vertexList);
        //qDebug() << "vertex = " << vertex;
        vertexList[vertex] = false;
        VertexHandle vhVertex = _mesh->vertex_handle(vertex);

        for (MyMesh::VertexVertexIter vv_it=_mesh->vv_iter(vhVertex); vv_it.is_valid(); vv_it++)
        {
            int poids = tabWeights[vertex] + 1;
            if(tabWeights[vv_it->idx()] > poids){
                //MAJ du poids
                tabWeights[vv_it->idx()] = poids;
                allPaths[vv_it->idx()] = vertex;
            }
        }
    }
}

/*------------------------------------------------------------------------------
 * Attention la fonction @dijkstra doit avoir été appelée avant.
 * Renvoit dans un tableau le chemin calculé entre
 * @vertexStart et @vertexEnd.
 * Met aussi à jour le chemin actuel @currentPath.
 * ----------------------------------------------------------------------------*/
vector<int> Dijkstra::calc_path(MyMesh *_mesh, int vertexEnd)
{
    if (allPaths.empty()) {
        qWarning() << "in" << __FUNCTION__ << ": allPaths is empty";
        exit(1);
    }
    vector<int> path;
    int vertexCur = vertexEnd;

    while (vertexCur != vertexStart)
    {
        VertexHandle vhCurrent = _mesh->vertex_handle(vertexCur);
        for (MyMesh::VertexEdgeIter ve_it=_mesh->ve_iter(vhCurrent); ve_it.is_valid(); ++ve_it)
        {
            EdgeHandle eh = *ve_it;
            HalfedgeHandle heha = _mesh->halfedge_handle(eh, 0);
            HalfedgeHandle hehb = _mesh->halfedge_handle(eh, 1);
            VertexHandle vha = _mesh->to_vertex_handle(heha);
            VertexHandle vhb = _mesh->to_vertex_handle(hehb);
            if (vha.idx()==allPaths[vertexCur]   ||  vhb.idx()==allPaths[vertexCur]) {
                path.push_back(eh.idx());
            }
        }
        vertexCur = allPaths[vertexCur];
    }
    currentPath = path;
    return path;
}







