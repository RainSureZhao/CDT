//
// Created by RainSure on 24-6-19.
//

#ifndef CDT_POINT_H
#define CDT_POINT_H

#include <iostream>
#include "numeric/numeric.h"

namespace cdt {

    template<typename T>
    class Point2d {
    public:
        Point2d() = default;
        Point2d(T x, T y) : x(x), y(y) {}
        Point2d(const Point2d &p) : x(p.x), y(p.y) {}
        Point2d(Point2d<T>&&) = default;

        T distance2(const Point2d &p) const;
        T distance(const Point2d &p) const;
        T norm2() const;

        Point2d& operator=(const Point2d<T>&) = default;
        Point2d& operator=(Point2d<T>&& ) = default;

        template<class U>
        operator Point2d<U>() {
            return Point2d<U>(U(x), U(y));
        }
        Point2d &operator+=(Point2d p) & {
            x += p.x;
            y += p.y;
            return *this;
        }
        Point2d &operator-=(Point2d p) & {
            x -= p.x;
            y -= p.y;
            return *this;
        }
        Point2d &operator*=(T v) & {
            x *= v;
            y *= v;
            return *this;
        }
        Point2d operator-() const {
            return Point2d(-x, -y);
        }
        friend Point2d operator+(Point2d a, Point2d b) {
            return a += b;
        }
        friend Point2d operator-(Point2d a, Point2d b) {
            return a -= b;
        }
        friend Point2d operator*(Point2d a, T b) {
            return a *= b;
        }
        friend Point2d operator*(T a, Point2d b) {
            return b *= a;
        }
        friend bool operator==(Point2d a, Point2d b) {
            return a.x == b.x && a.y == b.y;
        }
        friend std::istream &operator>>(std::istream &is, Point2d &p) {
            return is >> p.x >> p.y;
        }
        friend std::ostream &operator<<(std::ostream &os, Point2d p) {
            return os << "(" << p.x << ", " << p.y << ")";
        }
        friend bool almost_equal(Point2d a, Point2d b) {
            return almost_equal(a.x, b.x) && almost_equal(a.y, b.y);
        }

    public:
        T x, y;
    };

    template<typename T>
    class Point3d {
    public:
        Point3d() = default;
        Point3d(T x, T y, T z) : x(x), y(y), z(z) {}
        Point3d(const Point3d& p) = default;
        Point3d(Point3d<T>&&) = default;

        T distance2(const Point3d &p) const;
        T distance(const Point3d &p) const;
        T norm2() const;

        Point3d& operator=(const Point3d<T>&) = default;
        Point3d& operator=(Point3d<T>&& ) = default;

        template<class U>
        operator Point3d<U>() {
            return Point3d<U>(U(x), U(y), U(z));
        }
        Point3d &operator+=(Point3d p) & {
            x += p.x;
            y += p.y;
            z += p.z;
            return *this;
        }
        Point3d &operator-=(Point3d p) & {
            x -= p.x;
            y -= p.y;
            z -= p.z;
            return *this;
        }
        Point3d &operator*=(T v) & {
            x *= v;
            y *= v;
            z *= v;
            return *this;
        }
        Point3d operator-() const {
            return Point3d(-x, -y, -z);
        }
        friend Point3d operator+(Point3d a, Point3d b) {
            return a += b;
        }
        friend Point3d operator-(Point3d a, Point3d b) {
            return a -= b;
        }
        friend Point3d operator*(Point3d a, T b) {
            return a *= b;
        }
        friend Point3d operator*(T a, Point3d b) {
            return b *= a;
        }
        friend bool operator==(Point3d a, Point3d b) {
            return a.x == b.x && a.y == b.y && a.z == b.z;
        }
        friend std::istream &operator>>(std::istream &is, Point3d &p) {
            return is >> p.x >> p.y >> p.z;
        }
        friend std::ostream &operator<<(std::ostream &os, Point3d p) {
            return os << "(" << p.x << ", " << p.y << ", " << p.z << ")";
        }
        friend bool almost_equal(Point3d a, Point3d b) {
            return almost_equal(a.x, b.x) && almost_equal(a.y, b.y) && almost_equal(a.z, b.z);
        }

    public:
        T x, y, z;
    };

    template<typename T>
    using Vector2d = Point2d<T>;

    template<typename T>
    using Vector3d = Point3d<T>;

} // cdt

#endif //CDT_POINT_H
