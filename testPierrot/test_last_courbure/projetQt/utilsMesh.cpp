#include "utilsMesh.h"

#include <QDebug>

UtilsMesh::UtilsMesh()
{
    ;
}


////////////////////////    BOOL    ///////////////////////////////////////

bool UtilsMesh::edge_is_in_face(MyMesh *_mesh, int edgeID, int faceID)
{
    FaceHandle fh = _mesh->face_handle(faceID);

    for (MyMesh::FaceEdgeCWIter fe_it = _mesh->fe_cwiter(fh); fe_it.is_valid(); fe_it++)
    {
        EdgeHandle eh = *fe_it;
        if (eh.idx() == edgeID)
            return true;
    }

    return false;
}

bool UtilsMesh::faces_have_common_edge(MyMesh *_mesh, int face1ID, int face2ID)
{
    FaceHandle fh1 = _mesh->face_handle(face1ID);
    for (MyMesh::FaceEdgeCWIter fe_it = _mesh->fe_cwiter(fh1); fe_it.is_valid(); fe_it++)
    {
        EdgeHandle eh = *fe_it;
        if (edge_is_in_face(_mesh, eh.idx(), face2ID))
            return true;
    }
    return false;
}

bool UtilsMesh::vertex_is_in_face(MyMesh *_mesh, int faceID, int vertexID)
{
    FaceHandle fh = _mesh->face_handle(faceID);
    for (MyMesh::FaceVertexCWIter fv_it = _mesh->fv_cwiter(fh); fv_it.is_valid(); fv_it++)
    {
       VertexHandle visitor = *fv_it;
       if (vertexID == visitor.idx())   return true;
    }
    return false;
}


////////////////////////    HANDLES    /////////////////////////////////////

/*------------------------------------------------------------------------------
 * Trouve le vertex correspondant à la position @p.
 * @precision sert à choisir à quel point on approxime la position à trouver
 * ----------------------------------------------------------------------------*/
int UtilsMesh::get_vertex_of_point(MyMesh *_mesh, MyMesh::Point p, int precision)
{
    int numVertex = -1;
    float fPrec = 0.f;
    for (unsigned i=0; i<precision; i++) {
        fPrec += 0.01f;
    }

    for (MyMesh::VertexIter curVert = _mesh->vertices_begin(); curVert != _mesh->vertices_end(); curVert++)
    {
        VertexHandle vh = *curVert;
        MyMesh::Point pTest = _mesh->point(vh);
        //        qDebug() << QVector3D(pTest[0], pTest[1], pTest[2]);
        if (pTest[0] >= p[0] - fPrec    &&  pTest[0] <= p[0] + fPrec
                &&  pTest[1] >= p[1] - fPrec    &&  pTest[1] <= p[1] + fPrec
                &&  pTest[2] >= p[2] - fPrec    &&  pTest[2] <= p[2] + fPrec)
        {
            return vh.idx();
        }
    }

    return numVertex;
}

/*------------------------------------------------------------------------------
 * Renvoie l'id de la face opposée de @faceID par rapport à l'arete @edgeID
 * ATTENTION
 *  @edgeID doit appartenir à la face @faceID
 * ----------------------------------------------------------------------------*/
int UtilsMesh::get_opposite_edgeFace(MyMesh *_mesh, int faceID, int edgeID)
{
    FaceHandle fh;
    if (! UtilsMesh::edge_is_in_face(_mesh, edgeID, faceID)) {
        qWarning() << "Warning in " << __FUNCTION__ << ": @edgeID has to"
                                                       " be part of the face @faceID";
        return -1;
    }
    EdgeHandle eh = _mesh->edge_handle(edgeID);
    if (_mesh->is_boundary(eh)) return -1;

    MyMesh::HalfedgeHandle heh1 = _mesh->halfedge_handle(eh, 0);
    fh = _mesh->face_handle(heh1);
    if (fh.idx() != faceID) return fh.idx();

    MyMesh::HalfedgeHandle heh2 = _mesh->halfedge_handle(eh, 1);
    fh = _mesh->face_handle(heh2);
    if (fh.idx() != faceID) return fh.idx();

    return -1;
}

void UtilsMesh::get_vh_of_edge(MyMesh *_mesh, int edgeID,
                               VertexHandle &vh0, VertexHandle &vh1)
{
    EdgeHandle eh = _mesh->edge_handle(edgeID);
    MyMesh::HalfedgeHandle heh1 =  _mesh->halfedge_handle(eh, 0);
    //    MyMesh::HalfedgeHandle heh2 =  _mesh->halfedge_handle(eh, 1);
    vh0 = _mesh->from_vertex_handle(heh1);
    vh1 = _mesh->to_vertex_handle(heh1);
}

void UtilsMesh::get_eh_of_triangle_face(MyMesh *_mesh, int faceID,
                                        EdgeHandle &eh0, EdgeHandle &eh1, EdgeHandle &eh2)
{
    FaceHandle fh = _mesh->face_handle(faceID);
    int cpt = 1;
    for (MyMesh::FaceEdgeCWIter fe_it = _mesh->fe_cwiter(fh); fe_it.is_valid(); fe_it++)
    {
        if (cpt==1)         eh0 = *fe_it;
        else if (cpt==2)    eh1 = *fe_it;
        else if (cpt==3)    eh2 = *fe_it;
        else {
            qWarning() << "In " << __FUNCTION__ << ": Error cpt > 3 -> not a triangle!";
            exit (1);
        }
        cpt++;
    }
}

vector<VertexHandle> UtilsMesh::get_vh_of_face(MyMesh *_mesh, int faceID)
{
    vector<VertexHandle> vhs;

    FaceHandle fh = _mesh->face_handle(faceID);
    for (MyMesh::FaceVertexCWIter fv_it = _mesh->fv_cwiter(fh); fv_it.is_valid(); fv_it++)
    {
        VertexHandle vh = *fv_it;
        vhs.push_back(vh);
    }

    return vhs;
}

void UtilsMesh::get_vh_of_triangle_face(MyMesh *_mesh, int faceID,
                                        VertexHandle &vh0, VertexHandle &vh1, VertexHandle &vh2)
{
    FaceHandle fh = _mesh->face_handle(faceID);
    int cpt = 1;
    for (MyMesh::FaceVertexCWIter fv_it = _mesh->fv_cwiter(fh); fv_it.is_valid(); fv_it++)
    {
        if (cpt==1)         vh0 = *fv_it;
        else if (cpt==2)    vh1 = *fv_it;
        else if (cpt==3)    vh2 = *fv_it;
        else {
            qWarning() << "In " << __FUNCTION__ << ": Error cpt > 3 -> not a triangle!";
            exit (1);
        }
        cpt++;
    }
}

void UtilsMesh::get_points_of_triangle_face(MyMesh *_mesh, int faceID,
                                   MyMesh::Point &p1, MyMesh::Point &p2, MyMesh::Point &p3)
{
    FaceHandle fh = _mesh->face_handle(faceID);
    int cpt = 1;
    for (MyMesh::FaceVertexCWIter fv_it = _mesh->fv_cwiter(fh); fv_it.is_valid(); fv_it++)
    {
        VertexHandle vh = *fv_it;
        if (cpt==1)         p1 = _mesh->point(vh);
        else if (cpt==2)    p2 = _mesh->point(vh);
        else if (cpt==3)    p3 = _mesh->point(vh);
        else {
            qWarning() << "In " << __FUNCTION__ << ": Error cpt > 3 -> not a triangle!";
            exit (1);
        }
        cpt++;
    }
}

vector<FaceHandle> UtilsMesh::get_neighbour_faces(MyMesh *_mesh, int numFace)
{
    vector<FaceHandle> faces;
    FaceHandle fhBase = _mesh->face_handle(numFace);
    for (MyMesh::FaceFaceCWIter ff_it = _mesh->ff_cwiter(fhBase); ff_it.is_valid(); ff_it++)
    {
        FaceHandle fhTmp = *ff_it;
        faces.push_back(fhTmp);
    }
    return faces;
}

/*--------------------------------------------------------------
 * Simule un EdgeEdge Circulator en renvoyant la liste @ehs de
 * toutes les arêtes voisines de l'arête @numEdge
 * -----------------------------------------------------------*/
vector<EdgeHandle> UtilsMesh::get_edgeEdge_circulator(MyMesh *_mesh, int numEdge)
{
    if (static_cast<unsigned>(numEdge)>_mesh->n_edges()) {
        qWarning() << "In " << __FUNCTION__ << ": Error @numEdge not valid";
        exit (1);
    }
    vector<EdgeHandle> ehs;

    EdgeHandle eh = _mesh->edge_handle(numEdge);
    HalfedgeHandle heh = _mesh->halfedge_handle(eh, 0);
    VertexHandle vhTo = _mesh->to_vertex_handle(heh);
    VertexHandle vhFrom = _mesh->from_vertex_handle(heh);

    for (MyMesh::VertexEdgeCWIter ve_it = _mesh->ve_cwiter(vhTo); ve_it.is_valid(); ve_it++)
    {
        EdgeHandle ehTmp = *ve_it;
        if (ehTmp!=eh) ehs.push_back(ehTmp);
    }
    for (MyMesh::VertexEdgeCWIter ve_it = _mesh->ve_cwiter(vhFrom); ve_it.is_valid(); ve_it++)
    {
        EdgeHandle ehTmp = *ve_it;
        if (ehTmp!=eh) ehs.push_back(ehTmp);
    }

    return ehs;
}

EdgeHandle UtilsMesh::get_next_eh_of_vh(MyMesh *_mesh, int numVertex)
{
    VertexHandle vh = _mesh->vertex_handle(numVertex);
    EdgeHandle eh;
    for (MyMesh::VertexEdgeCWIter ve_it = _mesh->ve_cwiter(vh); ve_it.is_valid(); ve_it++)
    {
        eh = *ve_it;
        break;
    }
    return eh;
}

////////////////////////    TRANSFOS    ///////////////////////////////////

void UtilsMesh::add_triangle_face(MyMesh *_mesh,
                                  VertexHandle vh1,
                                  VertexHandle vh2,
                                  VertexHandle vh3)
{
    _mesh->add_face(vh1, vh2, vh3);
    FaceHandle fh = _mesh->face_handle(_mesh->n_faces()-1);
    _mesh->set_color(fh, MyMesh::Color(150, 150, 150));
}

void UtilsMesh::add_face(MyMesh *_mesh, vector<VertexHandle> vhs)
{
    vector<MyMesh::VertexHandle> uneNouvelleFace;
    for (auto vh : vhs)
    {
        uneNouvelleFace.push_back(vh);
    }
    _mesh->OpenMesh::PolyConnectivity::add_face(uneNouvelleFace);
    FaceHandle fh = _mesh->face_handle(_mesh->n_faces()-1);
    _mesh->set_color(fh, MyMesh::Color(150, 150, 150));
}

void UtilsMesh::collapseEdge(MyMesh* _mesh, int edgeID)
{
    EdgeHandle eh = _mesh->edge_handle(edgeID);
    MyMesh::HalfedgeHandle heh = _mesh->halfedge_handle(eh, 0);

    if (_mesh->is_collapse_ok(heh)) {
        MyMesh::VertexHandle vh0 = _mesh->to_vertex_handle(heh);
        MyMesh::VertexHandle vh1 = _mesh->from_vertex_handle(heh);
        MyMesh::Point newVertP = (_mesh->point(vh0) + _mesh->point(vh1))/2.f;
        _mesh->set_point(vh0, newVertP);
        _mesh->collapse(heh);
    }
    else {
        qDebug() << "in" << __FUNCTION__ << ":collapse edge not possible";
    }
    // permet de nettoyer le maillage et de garder la cohérence des indices après un collapse
    _mesh->garbage_collection();
}

void UtilsMesh::flipEdge(MyMesh *_mesh, int edgeID)
{
    EdgeHandle eh = _mesh->edge_handle(edgeID);
    if(_mesh->is_flip_ok(eh))
        _mesh->flip(eh);
}

void UtilsMesh::splitEdge(MyMesh *_mesh, int edgeID, bool bCopy)
{
    MyMesh::Point P = UtilsMesh::middle_edge(_mesh, edgeID);
    VertexHandle vh = _mesh->add_vertex(P);
    EdgeHandle eh = _mesh->edge_handle(edgeID);
    if (!bCopy) _mesh->TriConnectivity::split_edge(eh, vh);
    else        _mesh->TriConnectivity::split_edge_copy(eh, vh);
}


////////////////////////    AUTRES    /////////////////////////////////////

MyMesh::Point UtilsMesh::barycentre_triangle(MyMesh *_mesh, int faceID)
{
    MyMesh::Point _p1, _p2, _p3;
    get_points_of_triangle_face(_mesh, faceID, _p1, _p2, _p3);

    MyMesh::Point P;
    P  = _p1;
    P += _p2;
    P += _p3;
    P /= 3;

    return P;
}

QVector3D UtilsMesh::to_qvector3D(MyMesh::Point p)
{
    return QVector3D(p[0], p[1], p[2]);
}

float UtilsMesh::distance_point_to_face(MyMesh *_mesh, MyMesh::Point p, int faceID)
{
    MyMesh::Point _P = UtilsMesh::barycentre_triangle(_mesh, faceID);
    QVector3D P = UtilsMesh::to_qvector3D(_P);
    return P.distanceToPoint(UtilsMesh::to_qvector3D(p));
}

float  UtilsMesh::middle_edge_coord(float p1, float p2)
{
    float dist = fabs(p1 - p2);
    float res;
    if (p1 <= p2) {
        res = p1 + (dist/2.f);
    }
    else {
        res = p1 -(dist/2.f);
    }
    return res;
}


/*--------------------------------------------------------------------------
 *  Renvoie la positon du milieu d'une arête d'indice @edgeID
 * -------------------------------------------------------------------------*/
MyMesh::Point UtilsMesh::middle_edge(MyMesh *_mesh, int edgeID)
{
    EdgeHandle eh = _mesh->edge_handle(edgeID);

    MyMesh::HalfedgeHandle heh1 = _mesh->halfedge_handle(eh, 0);
    MyMesh::HalfedgeHandle heh2 = _mesh->halfedge_handle(eh, 1);
    VertexHandle vh1 = _mesh->to_vertex_handle(heh1);
    VertexHandle vh2 = _mesh->to_vertex_handle(heh2);
    MyMesh::Point p1 = _mesh->point(vh1);
    MyMesh::Point p2 = _mesh->point(vh2);

    float X = UtilsMesh::middle_edge_coord(p1[0], p2[0]);
    float Y = UtilsMesh::middle_edge_coord(p1[1], p2[1]);
    float Z = UtilsMesh::middle_edge_coord(p1[2], p2[2]);
    MyMesh::Point myP(X, Y, Z);
    return myP;
}


