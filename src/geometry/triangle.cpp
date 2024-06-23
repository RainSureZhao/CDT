//
// Created by RainSure on 24-6-19.
//

#include "geometry/triangle.h"
#include "common.h"
#include <cassert>

namespace cdt {

    template<typename T>
    bool Triangle2d<T>::containsVertex(const Point2d<T> &point) const {
        return almost_equal(this->a, point) || almost_equal(this->b, point) || almost_equal(this->c, point);
    }

    template<typename T>
    bool Triangle2d<T>::circumCircleContains(const Point2d<T> &point) const {
        auto ab = a.norm2();
        auto cd = b.norm2();
        auto ef = c.norm2();

        auto ax = a.x;
        auto ay = a.y;
        auto bx = b.x;
        auto by = b.y;
        auto cx = c.x;
        auto cy = c.y;

        auto circum_x = (ab * (cy - by) + cd * (ay - cy) + ef * (by - ay)) / (ax * (cy - by) + bx * (ay - cy) + cx * (by - ay));
        auto circum_y = (ab * (cx - bx) + cd * (ax - cx) + ef * (bx - ax)) / (ay * (cx - bx) + by * (ax - cx) + cy * (bx - ax));

        Point2d<T> circum(circum_x / 2, circum_y / 2);
        const T circum_radius = a.distance2(circum);
        auto dist = point.distance2(circum);
        return dist <= circum_radius;
    }

    template<typename T>
    bool operator==(Triangle2d<T> lhs, Triangle2d<T> rhs) {
        return (lhs.a == rhs.a && lhs.b == rhs.b && lhs.c == rhs.c) ||
               (lhs.a == rhs.a && lhs.b == rhs.c && lhs.c == rhs.b) ||
               (lhs.a == rhs.b && lhs.b == rhs.a && lhs.c == rhs.c) ||
               (lhs.a == rhs.b && lhs.b == rhs.c && lhs.c == rhs.a) ||
               (lhs.a == rhs.c && lhs.b == rhs.a && lhs.c == rhs.b) ||
               (lhs.a == rhs.c && lhs.b == rhs.b && lhs.c == rhs.a);
    }

    template<typename T>
    std::ostream& operator<<(std::ostream &os, Triangle2d<T>& tri) {
        os << "Triangle2d: " << tri.a << " " << tri.b << " " << tri.c;
    }

    template<typename T>
    bool almost_equal(Triangle2d<T> lhs, Triangle2d<T> rhs) {
        return almost_equal(lhs.a, rhs.a) && almost_equal(lhs.b, rhs.b) && almost_equal(lhs.c, rhs.c) ||
               almost_equal(lhs.a, rhs.a) && almost_equal(lhs.b, rhs.c) && almost_equal(lhs.c, rhs.b) ||
               almost_equal(lhs.a, rhs.b) && almost_equal(lhs.b, rhs.a) && almost_equal(lhs.c, rhs.c) ||
               almost_equal(lhs.a, rhs.b) && almost_equal(lhs.b, rhs.c) && almost_equal(lhs.c, rhs.a) ||
               almost_equal(lhs.a, rhs.c) && almost_equal(lhs.b, rhs.a) && almost_equal(lhs.c, rhs.b) ||
               almost_equal(lhs.a, rhs.c) && almost_equal(lhs.b, rhs.b) && almost_equal(lhs.c, rhs.a);
    }

    /// Next triangle adjacent to a vertex (clockwise)
    /// @returns pair of next triangle and the other vertex of a common edge
    template<typename T>
    std::pair<cdt::TriangleIndex, cdt::VertexIndex> Triangle2d<T>::next(cdt::VertexIndex i) const {
        assert(vertices[0] == i or vertices[1] == i or vertices[2] == i);
        if(vertices[0] == i) {
            return std::make_pair(neighbors[0], vertices[1]);
        } else if(vertices[1] == i) {
            return std::make_pair(neighbors[1], vertices[2]);
        } else {
            return std::make_pair(neighbors[2], vertices[0]);
        }
    }

    /// Previous triangle adjacent to a vertex (counter-clockwise)
    /// @returns pair of previous triangle and the other vertex of a common edge
    template<typename T>
    std::pair<TriangleIndex, VertexIndex> Triangle2d<T>::prev(VertexIndex i) const {
        assert(vertices[0] == i or vertices[1] == i or vertices[2] == i);
        if(vertices[0] == i) {
            return std::make_pair(neighbors[2], vertices[2]);
        } else if(vertices[1] == i) {
            return std::make_pair(neighbors[0], vertices[0]);
        } else {
            return std::make_pair(neighbors[1], vertices[1]);
        }
    }

    template<typename T>
    bool Triangle2d<T>::containsVertex(VertexIndex i) const {
        return std::find(std::begin(vertices), std::end(vertices), i) != std::end(vertices);
    }

    template class Triangle2d<float>;
    template class Triangle2d<double>;

} // cdt