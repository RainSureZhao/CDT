//
// Created by RainSure on 24-6-20.
//

#ifndef CDT_DRAW_H
#define CDT_DRAW_H

#include <vector>
#include <map>
#include "geometry/triangle.h"
#include <CGAL/Exact_predicates_inexact_constructions_kernel.h>
#include <CGAL/Surface_mesh.h>
#include <CGAL/draw_surface_mesh.h>
typedef CGAL::Exact_predicates_inexact_constructions_kernel K;
typedef K::Point_3                                          Point;
typedef CGAL::Surface_mesh<Point>                           Mesh;                                       Point;

namespace cdt {
    template<typename T>
    Mesh drawMesh(const std::vector<cdt::Triangle2d<T>>& triangles) {
        std::map<cdt::Point2d<T>, Mesh::Vertex_index> mp;
        Mesh mesh;
        for(auto &triangle : triangles) {
            auto a = triangle.a, b = triangle.b, c = triangle.c;
            if(mp.find(a) == mp.end()) {
                mp[a] = mesh.add_vertex(Point(a.x, a.y, 0));
            }
            if(mp.find(b) == mp.end()) {
                mp[b] = mesh.add_vertex(Point(b.x, b.y, 0));
            }
            if(mp.find(c) == mp.end()) {
                mp[c] = mesh.add_vertex(Point(c.x, c.y, 0));
            }
            mesh.add_face(mp[a], mp[b], mp[c]);
        }
        CGAL::draw(mesh);
        return mesh;
    }
}

#endif //CDT_DRAW_H
