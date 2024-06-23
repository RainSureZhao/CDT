//
// Created by RainSure on 24-6-19.
//

#ifndef CDT_TRIANGLE_H
#define CDT_TRIANGLE_H

#include "geometry/point.h"
#include "common.h"

namespace cdt {

    /// Triangulation triangle (counter-clockwise winding)
    /*
     *      v3
     *      /\
     *   n3/  \n2
     *    /____\
     *  v1  n1  v2
     */
    template<typename T>
    class Triangle2d {
    public:
        Triangle2d() = default;
        Triangle2d(const Point2d<T>& a, const Point2d<T>& b, const Point2d<T>& c) : a(a), b(b), c(c) {
        }
        Triangle2d(const Triangle2d&) = default;
        Triangle2d(Triangle2d&&) = default;

        Triangle2d(const VertexIndexArray3& vertices, const NeighborIndexArray3& neighbors) : vertices(vertices), neighbors(neighbors) {
        }

        [[nodiscard]] std::pair<TriangleIndex, VertexIndex> next(VertexIndex i) const;
        [[nodiscard]] std::pair<TriangleIndex, VertexIndex> prev(VertexIndex i) const;
        [[nodiscard]] bool containsVertex(VertexIndex i) const;

        Triangle2d& operator=(const Triangle2d&) = default;
        Triangle2d& operator=(Triangle2d&&) = default;

        friend bool operator==(Triangle2d lhs, Triangle2d rhs);
        bool containsVertex(const Point2d<T>& point) const;
        bool circumCircleContains(const Point2d<T>& point) const;

        friend std::ostream &operator<<(std::ostream &os, Triangle2d& tri);
        friend bool almost_equal(Triangle2d lhs, Triangle2d rhs);
    public:
        Point2d<T> a, b, c;
        VertexIndexArray3 vertices {noVertex, noVertex, noVertex};;  /// 三角形的三个顶点索引
        NeighborIndexArray3 neighbors {noNeighbor, noNeighbor, noNeighbor};  /// 三角形的三个邻居三角形索引
        bool isBad = false;
    };

} // cdt

#endif //CDT_TRIANGLE_H
