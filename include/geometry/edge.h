//
// Created by RainSure on 24-6-19.
//

#ifndef CDT_EDGE_H
#define CDT_EDGE_H

#include "geometry/point.h"
#include "numeric/numeric.h"
#include "common.h"

namespace cdt {
    template<typename T>
    class Edge2d {
    public:
        Edge2d() = default;
        Edge2d(const Edge2d&) = default;
        Edge2d(Edge2d&&) = default;
        Edge2d(const Point2d<T> &a, const Point2d<T> &b) : a(a), b(b) {}
        Edge2d(const VertexIndex& v1, const VertexIndex& v2) : verts(std::make_pair(v1, v2)) {}

        Edge2d& operator=(const Edge2d&) = default;

        friend bool operator==(Edge2d lhs, Edge2d rhs) {
            return lhs.a == rhs.a && lhs.b == rhs.b;
        }

        friend std::ostream &operator<<(std::ostream &os, Edge2d& p) {
            return os << p.a << ", " << p.b;
        }
        friend bool almost_equal(Edge2d lhs, Edge2d rhs) {
            return almost_equal(lhs.a, rhs.a) && almost_equal(lhs.b, rhs.b) || almost_equal(lhs.a, rhs.b) && almost_equal(lhs.b, rhs.a);
        }
    public:
        Point2d<T> a, b;
        std::pair<VertexIndex, VertexIndex> verts;
        bool isBad = false;
    };
}

#endif //CDT_EDGE_H
