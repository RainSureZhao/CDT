//
// Created by RainSure on 24-6-19.
//

#include "geometry/triangle.h"

namespace cdt {

    template<typename T>
    bool Triangle2d<T>::containsVertex(const Point2d<T> &point) const {
        return almost_equal(this->a, point) || almost_equal(this->b, point) || almost_equal(this->c, point);
    }

    template<typename T>
    bool Triangle2d<T>::circumCircleContains(const Point2d<T> &point) const {
        // 计算出三个顶点到原点的距离的平方
        auto oa = a.norm2();
        auto ob = b.norm2();
        auto oc = b.norm2();

        auto ax = a.x, ay = a.y, bx = b.x, by = b.y, cx = c.x, cy = c.y;

        // 计算圆心坐标
        auto circum_x = (oa * (cy - by) + ob * (ay - cy) + oc * (by - ay)) / (ax * (cy - by) + bx * (ay - cy) + cx * (by - ay));
        auto circum_y = (oa * (cx - bx) + ob * (ax - cx) + oc * (bx - ax)) / (ay * (cx - bx) + by * (ax - cx) + cy * (bx - ax));

        Point2d<T> center{circum_x / 2, circum_y / 2};
        auto radius = a.distance2(center);
        auto dist = point.distance2(center);
        // 零误差
        return dist <= radius;
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

    template class Triangle2d<float>;
    template class Triangle2d<double>;

} // cdt