#ifndef CONTOUR_H
#define CONTOUR_H

#include <OpenMesh/Core/IO/MeshIO.hh>
#include <OpenMesh/Core/Mesh/TriMesh_ArrayKernelT.hh>

using namespace OpenMesh;
using namespace OpenMesh::Attributes;

#include <fstream>
#include <QVector3D>

#include "utilsMesh.h"
#include "selectedpoints.h"
#include "livewire.h"

#include <vector>
#include <qdebug.h>

using namespace std;

class Contour
{
public:
    /**
     * \brief Constructeur
     *
     * Constructeur de la classe LiveWire
     *
     * \param _vertices : le contour d'id de sommets
     */
    Contour(MyMesh &_mesh);
    Contour(MyMesh &_mesh, unsigned _begin);
    Contour(MyMesh &_mesh, vector<unsigned> _vertices);
    /**
     * \brief Constructeur
     *
     * Constructeur de la classe LiveWire chargeant un fichier .xyz comme contour.
     * (éviter de mettre les données de couleur dans le fichier).
     *
     * \param path : le contour contenu dans un fichier .xyz
     */
    Contour(MyMesh &_mesh, char *path);

    unsigned get_start();
    unsigned get_end();
    vector<unsigned> get_contour();
    void set_contour(vector<unsigned> tmp);

    /**
     * \brief Réduit le nombre de sommets
     *
     * Réduit le nombre de sommets dans @verticesContour
     * à partir d'une valeur modulo.
     * (exemple: on ne garde que les id de sommets (dans le tableau@verticesContour)
     * dont le modulo @moduloVal vaut 0)
     *
     * \param modulo : pour indiquer la proprtiuon de sommets à conserver
     */
    void reduct(int modulo);

    void add_edge(unsigned numEdge);
    /**
     * \brief Ajoute un sommet dans le contour
     *
     * \param numVertex : l'id du sommet
     */
    void add_vertex(unsigned numVertex);
    /**
     * \brief dessine le contour dans le viewer
     *
     * Appelle le livewire pour chaque couple de sommets dans le contour
     * et dessine les arêtes dans le viewer.
     *
     * \param _mesh : le maillage.
     * \param _sightPoint : le point de vue utilisateur.
     */
    void draw_contour(MyMesh *_mesh, MyMesh::Point _sightPoint);

    //    MyMesh load_points_with_mesh(char *path);

    double half_thickness = 2; //Radius of pre-supposed brush radius which "draws" the stroke

    void display(int profDisplay, bool flagColor=false);

protected:

    MyMesh &mesh;

    int startPoint; /*!< Indice du premier sommet du contour */
    int endPoint;   /*!< Indice du dernier sommet du contour */
    vector<unsigned> edgesContour;
    vector<unsigned> verticesContour; /*!< Le contour de sommets */

    /**
     * \brief charge un nuage
     *
     * Charge un fichier .txt ou .xyz à partir de @filename
     * (attention ne gère pas encore les couleurs).
     *
     * \param filename : le fichier (.xyz ou .txt) contenant les données d'un nuage.
     * \param _sightPoint : le point de vue utilisateur.
     *
     * \return les coordonnées du nuage
     */
    vector<QVector3D> loadCloud(const string &filename);

    /**
     * \brief cherche la dimension maximale
     *
     * Cherche le point dans @tmp qui a, en fonction du boolean @b_max,
     * la @dim maximale ou minimale
     *
     * \param tmp : contour d'id de sommets.
     * \param dim : 0,1 ou 2 aka x, y ou z.
     *
     * \return l'id du sommet de coord min
     */
    int search_borne_dim(vector<int> tmp, int dim, bool b_max=true);

    /**
     * \brief Cherche le sommet dont l'indice est dans @tmp, qui est le plus proche du sommet d'indice @id.
     *
     * Cherche le sommet dont l'indice est dans @tmp,
     * qui est le plus proche du sommet d'indice @id.
     * Le sommet dans @tmp n'est reconnu que s'il n'est pas dans
     * @verticesContour.
     * Retourne -1 si aucun sommet compatible.
     *
     * \param tmp : contour d'id de sommets.
     * \param id : id d'un sommet.
     *
     * \return l'id du sommet trouvé ou -1 si aucun sommet compatible.
     */
    int search_min_dist_vertex_from_vertex(vector<int> tmp, int id);
};

#endif // CONTOUR_H
