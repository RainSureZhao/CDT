//
// Created by RainSure on 24-6-19.
//

#ifndef CDT_TRIANGLE_H
#define CDT_TRIANGLE_H

#include "geometry/point.h"

namespace cdt {

    template<typename T>
    class Triangle2d {
    public:
        Triangle2d() = default;
        Triangle2d(const Point2d<T>& a, const Point2d<T>& b, const Point2d<T>& c) : a(a), b(b), c(c) {}
        Triangle2d(const Triangle2d&) = default;
        Triangle2d(Triangle2d&&) = default;

        Triangle2d& operator=(const Triangle2d&) = default;
        Triangle2d& operator=(Triangle2d&&) = default;

        friend bool operator==(Triangle2d lhs, Triangle2d rhs);
        bool containsVertex(const Point2d<T>& point) const;
        bool circumCircleContains(const Point2d<T>& point) const;

        friend std::ostream &operator<<(std::ostream &os, Triangle2d& tri);
        friend bool almost_equal(Triangle2d lhs, Triangle2d rhs);
    public:
        Point2d<T> a, b, c;
        bool isBad = false;
    };

} // cdt

#endif //CDT_TRIANGLE_H
