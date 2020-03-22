#ifndef RAYCASTING_H
#define RAYCASTING_H
#include <QVector3D>
#include <QVector>

inline namespace Raycasting
{
    static uint32_t TEST_CULL = 0;

    void enable_backface_culling();

    void disable_backface_culling();

    /**
     * \brief Intersection Algorithm based on Moller-Trumbore Method
     *
     *
     */
    bool intersects_triangle(QVector3D origin,
                                    QVector3D direction,
                                    QVector<QVector3D> faceVertices,
                                    QVector3D& outIntersectionPoint);
}

#endif // RAYCASTING_H
