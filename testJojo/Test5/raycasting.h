#ifndef RAYCASTING_H
#define RAYCASTING_H
#include <QVector3D>
#include <QVector>

namespace Raycasting
{
    static uint32_t TEST_CULL = 0;

    inline void enable_backface_culling(void);

    inline void disable_backface_culling(void);

    /**
     * \brief Intersection Algorithm based on Moller-Trumbore Method
     *
     *
     */
    inline bool intersects_triangle(QVector3D origin,
                                    QVector3D direction,
                                    QVector<QVector3D> faceVertices,
                                    QVector3D& outIntersectionPoint);
}

#endif // RAYCASTING_H
