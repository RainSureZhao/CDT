//
// Created by RainSure on 24-6-19.
//

#include "geometry/point.h"
#include <cmath>

namespace cdt {
    template<typename T>
    T Point2d<T>::distance2(const Point2d<T> &p) const {
        T dx = x - p.x;
        T dy = y - p.y;
        return dx * dx + dy * dy;
    }

    template<>
    float Point2d<float>::distance(const cdt::Point2d<float> &p) const {
        return std::hypotf(x - p.x, y - p.y);
    }

    template<>
    double Point2d<double>::distance(const cdt::Point2d<double> &p) const {
        return std::hypot(x - p.x, y - p.y);
    }

    template<typename T>
    T Point2d<T>::norm2() const {
        return x * x + y * y;
    }

    template class Point2d<float>;
    template class Point2d<double>;


    template<typename T>
    T Point3d<T>::distance2(const Point3d<T> &p) const {
        T dx = x - p.x;
        T dy = y - p.y;
        T dz = z - p.z;
        return dx * dx + dy * dy + dz * dz;
    }

    template<>
    float Point3d<float>::distance(const cdt::Point3d<float> &p) const {
        return std::hypot(x - p.x, y - p.y, z - p.z);
    }

    template<>
    double Point3d<double>::distance(const cdt::Point3d<double> &p) const {
        return std::hypot(x - p.x, y - p.y, z - p.z);
    }

    template<typename T>
    T Point3d<T>::norm2() const {
        return x * x + y * y + z * z;
    }

    template class Point3d<float>;
    template class Point3d<double>;

} // cdt