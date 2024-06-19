//
// Created by RainSure on 24-6-19.
//

#include "dt/delaunay2d.h"

namespace cdt {
    /**
     * function BowyerWatson (pointList)
      // pointList is a set of coordinates defining the points to be triangulated
      triangulation := empty triangle mesh data structure
      add super-triangle to triangulation // must be large enough to completely contain all the points in pointList
      for each point in pointList do // add all the points one at a time to the triangulation
         badTriangles := empty set
         for each triangle in triangulation do // first find all the triangles that are no longer valid due to the insertion
            if point is inside circumcircle of triangle
               add triangle to badTriangles
         polygon := empty set
         for each triangle in badTriangles do // find the boundary of the polygonal hole
            for each edge in triangle do
               if edge is not shared by any other triangles in badTriangles
                  add edge to polygon
         for each triangle in badTriangles do // remove them from the data structure
            remove triangle from triangulation
         for each edge in polygon do // re-triangulate the polygonal hole
            newTri := form a triangle from edge to point
            add newTri to triangulation
      for each triangle in triangulation // done inserting points, now clean up
         if triangle contains a vertex from original super-triangle
            remove triangle from triangulation
      return triangulation
     *
     *
     *
     * @tparam T
     * @param points
     * @return
     */
    template<typename T>
    const std::vector<Triangle2d<T>> &Delaunay2d<T>::triangulate(std::vector<Point2d<T>> &points) {
        _vertices = points;

        // 首先生成一个足够大的三角形，保证所有顶点都能够在里面
        auto minX = _vertices[0].x, maxX = _vertices[0].x, minY = _vertices[0].y, maxY = _vertices[0].y;
        for(std::size_t i = 0; i < _vertices.size(); i ++) {
            minX = std::min(minX, _vertices[i].x);
            maxX = std::max(maxX, _vertices[i].x);
            minY = std::min(minY, _vertices[i].y);
            maxY = std::max(maxY, _vertices[i].y);
        }

        auto dx = maxX - minX;
        auto dy = maxY - minY;
        auto deltaMax = std::max(dx, dy);
        auto midX = (minX + maxX) / 2;
        auto midY = (minY + maxY) / 2;

        Point2d<T> p1{midX - 20 * deltaMax, midY - deltaMax}; // 左下角
        Point2d<T> p2{midX, midY + 20 * deltaMax};            // 上顶点
        Point2d<T> p3{midX + 20 * deltaMax, midY - deltaMax}; // 右下角

        if(!_triangles.empty()) {
            _triangles.clear();
        }
        if(!_edges.empty()) {
            _edges.clear();
        }

        _triangles.emplace_back(p1, p2, p3);

        for(std::size_t i = 0; i < _vertices.size(); i ++) {
            auto& p = _vertices[i];
            std::vector<Edge2d<T>> polygon;
            // 检查当前已经生成的所有三角形，判断是否不满足Delaunay条件
            for(auto& t : _triangles) {
                if(t.circumCircleContains(p)) {
                    t.isBad = true;
                    polygon.push_back(Edge2d<T>(t.a, t.b));
                    polygon.push_back(Edge2d<T>(t.b, t.c));
                    polygon.push_back(Edge2d<T>(t.c, t.a));
                }
            }

            // 删除所有被标记的三角形
            _triangles.erase(std::remove_if(std::begin(_triangles), std::end(_triangles), [](auto &t) {
                return t.isBad;
            }), std::end(_triangles));

            // 标记共享边
            for(auto it1 = std::begin(polygon); it1 != std::end(polygon); it1 ++) {
                for(auto it2 = it1 + 1; it2 != std::end(polygon); it2 ++) {
                    if(almost_equal(*it1, *it2)) {
                        it1->isBad = true;
                        it2->isBad = true;
                    }
                }
            }

            // 删除共享边
            polygon.erase(std::remove_if(std::begin(polygon), std::end(polygon), [](auto &e) {
                return e.isBad;
            }), std::end(polygon));

            // 生成新的三角形
            for(const auto& e : polygon) {
                _triangles.push_back(Triangle2d<T>(e.a, e.b, p));
            }
        }

        // 删除一开始生成的三个顶点相关的三角形
        _triangles.erase(std::remove_if(std::begin(_triangles), std::end(_triangles), [](auto& t) {
            return t.isBad;
        }), std::end(_triangles));

        for(const auto& tri : _triangles) {
            _edges.push_back({tri.a, tri.b});
            _edges.push_back({tri.b, tri.c});
            _edges.push_back({tri.c, tri.a});
        }

        return _triangles;
    }

    template<typename T>
    const std::vector<Triangle2d<T>> &Delaunay2d<T>::getTriangles() const {
        return _triangles;
    }

    template<typename T>
    const std::vector<Edge2d<T>> &Delaunay2d<T>::getEdges() const {
        return _edges;
    }

    template<typename T>
    const std::vector<Point2d<T>> &Delaunay2d<T>::getVertices() const {
        return _vertices;
    }

    template class Delaunay2d<float>;
    template class Delaunay2d<double>;

} // cdt