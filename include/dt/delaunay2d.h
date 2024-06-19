//
// Created by RainSure on 24-6-19.
//

#ifndef CDT_DELAUNAY2D_H
#define CDT_DELAUNAY2D_H

#include "geometry/edge.h"
#include "geometry/point.h"
#include "geometry/triangle.h"
#include <algorithm>
#include <vector>

namespace cdt {

    template<typename T>
    class Delaunay2d {
    public:
        Delaunay2d() = default;
        Delaunay2d(const Delaunay2d& ) = delete;
        Delaunay2d(Delaunay2d&& ) = delete;
        Delaunay2d& operator=(const Delaunay2d&) = delete;
        Delaunay2d& operator=(Delaunay2d&&) = delete;


        const std::vector<Triangle2d<T>>& triangulate(std::vector<Point2d<T>>& points);
        const std::vector<Triangle2d<T>>& getTriangles() const;
        const std::vector<Edge2d<T>>& getEdges() const;
        const std::vector<Point2d<T>>& getVertices() const;

    private:
        std::vector<Triangle2d<T>> _triangles;
        std::vector<Edge2d<T>> _edges;
        std::vector<Point2d<T>> _vertices;
    };

} // cdt

#endif //CDT_DELAUNAY2D_H
