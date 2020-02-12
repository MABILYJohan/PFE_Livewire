#include "raycasting.h"

// (Backface) Culling allows to
extern uint32_t TEST_CULL;

void Raycasting::enable_backface_culling()
{
    if(!TEST_CULL)
        TEST_CULL = 1;
}

void Raycasting::disable_backface_culling()
{
    if(TEST_CULL)
        TEST_CULL = 0;
}

bool Raycasting::intersects_triangle(QVector3D origin,
                                     QVector3D direction,
                                     QVector<QVector3D> faceVertices,
                                     QVector3D& out_intersection_point)
{
    const float EPSILON = 0.0000001;

    double t, u, v;

    QVector3D edge1, edge2, tvec, pvec, qvec;
    double det, inv_det;

    edge1 = faceVertices[1] - faceVertices[0];
    edge2 = faceVertices[2] - faceVertices[0];

    pvec = QVector3D::crossProduct(direction,edge2);

    det = QVector3D::dotProduct(edge1, pvec);

    if(TEST_CULL)
    {
        if (det < EPSILON)
            return false;

        tvec = origin-faceVertices[0];

        u = QVector3D::dotProduct(tvec,pvec);

        if(u < 0.0 || u > det)
        {
            return false;
        }

        qvec = QVector3D::crossProduct(tvec, edge1);
        v = QVector3D::dotProduct(direction,qvec);

        if(v < 0.0 || (u+v) > det)
        {
            return false;
        }

        t = QVector3D::dotProduct(edge2, qvec);
        inv_det = 1.0/det;

        t *= inv_det;
        u *= inv_det;
        v *= inv_det;
    }

    else
    {
        if(det > -EPSILON && det < EPSILON)
            return false;

        inv_det = 1.0 / det;

        tvec = origin - faceVertices[0];

        u = QVector3D::dotProduct(tvec,pvec) * inv_det;
        if(u < 0.0 || u > 1.0)
            return false;

        qvec = QVector3D::crossProduct(tvec, edge1);

        v = QVector3D::dotProduct(direction, qvec) * inv_det;
        if(v < 0.0 || (u + v) > 1.0)
        {
            return false;
        }

        t = QVector3D::dotProduct(edge2, qvec) * inv_det;

    }

    out_intersection_point.setX(u);
    out_intersection_point.setY(v);
    out_intersection_point.setZ(t);

    return true;

}
