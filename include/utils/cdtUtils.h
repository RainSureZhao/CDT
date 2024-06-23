//
// Created by RainSure on 24-6-22.
//

#ifndef CONSTRAINTDELAUNAYTRIANGULATION_CDTUTILS_H
#define CONSTRAINTDELAUNAYTRIANGULATION_CDTUTILS_H

#include <cassert>
#include "geometry/edge.h"
#include "geometry/triangle.h"
#include "common.h"
namespace cdt {
    template<typename T>
    VertexIndex edge_get_v1(const Edge2d<T>& e) {
        return e.verts.first;
    }

    template<typename T>
    VertexIndex edge_get_v2(const Edge2d<T>& e) {
        return e.verts.second;
    }

    template<typename T>
    Edge2d<T> edge_make(const VertexIndex& v1, const VertexIndex& v2) {
        return Edge2d<T>(v1, v2);
    }

    ///  Advance vertex or neighbor index counter-clockwise
    VertexIndex ccw(VertexIndex i) {
        return (i + 1) % 3;
    }

    /// Advance vertex or neighbor index clockwise
    VertexIndex cw(VertexIndex i) {
        return (i + 2) % 3;
    }

    class PointTriangleLocation {
    public:
        enum Enum {
            Inside,
            Outside,
            OnEdge1,
            OnEdge2,
            OnEdge3,
            OnVertex,
        };
    };

    class PointLineLocation {
    public:
        enum Enum {
            Left,
            Right,
            OnLine,
        };
    };

    bool IsOnEdge(const PointTriangleLocation::Enum location) {
        return location == PointTriangleLocation::OnEdge1 || location == PointTriangleLocation::OnEdge2 || location == PointTriangleLocation::OnEdge3;
    }

    template<typename T>
    PointLineLocation::Enum LocatePointLine(const Point2d<T>& point, const Point2d<T>& v1, const Point2d<T>& v2) {
        auto result = PointLineLocation::Left;
        auto det = (v2.x - v1.x) * (point.y - v1.y) - (point.x - v1.x) * (v2.y - v1.y);
        if (det > 0) {
            result = PointLineLocation::Right;
        } else if (det == 0) {
            result = PointLineLocation::OnLine;
        }
        return result;
    }

    template<typename T>
    PointTriangleLocation::Enum LocatePointTriangle(const Point2d<T>& point, const Point2d<T>& a, const Point2d<T>& b, const Point2d<T>& c) {
        auto result = PointTriangleLocation::Inside;
        auto edgeCheck = LocatePointLine(point, a, b);
        if(edgeCheck == PointLineLocation::Right) {
            return PointTriangleLocation::Outside;
        } else if(edgeCheck == PointLineLocation::OnLine) {
            result = PointTriangleLocation::OnEdge1;
        }
        edgeCheck = LocatePointLine(point, b, c);
        if(edgeCheck == PointLineLocation::Right) {
            return PointTriangleLocation::Outside;
        }
        if(edgeCheck == PointLineLocation::OnLine) {
            result = (result == PointTriangleLocation::Inside) ? PointTriangleLocation::OnEdge2 : PointTriangleLocation::OnVertex;
        }
        edgeCheck = LocatePointLine(point, c, a);
        if (edgeCheck == PointLineLocation::Right) {
            return PointTriangleLocation::Outside;
        }
        if (edgeCheck == PointLineLocation::OnLine) {
            result = (result == PointTriangleLocation::Inside) ? PointTriangleLocation::OnEdge3 : PointTriangleLocation::OnVertex;
        }
        return result;
    }

    int EdgeNeighbor(const PointTriangleLocation::Enum location) {
        assert(IsOnEdge(location));
        return location - PointTriangleLocation::OnEdge1;
    }

    int opposedVertexIndex(const NeighborIndexArray3& neighbors, TriangleIndex iTopo) {
        assert(neighbors[0] == iTopo || neighbors[1] == iTopo || neighbors[2] == iTopo);
        if(neighbors[0] == iTopo) {
            return 2;
        } else if(neighbors[1] == iTopo) {
            return 0;
        } else {
            return 1;
        }
    }

    template<typename T>
    T CalculateDET(const T x11, const T x12, const T x13, const T x21, const T x22, const T x23, const T x31, const T x32, const T x33) {
        return x11 * (x22 * x33 - x23 * x32) - x12 * (x21 * x33 - x23 * x31) + x13 * (x21 * x32 - x22 * x31);
    }

    template<typename T>
    bool InCircle(const T ax, const T ay, const T bx, const T by, const T cx, const T cy, const T dx, const T dy) {
        // if d lies inside the oriented circle abc return a positive value
        T det = CalculateDET(bx, by, bx * bx + by * by, cx, cy, cx * cx + cy * cy, dx, dy, dx * dx + dy * dy)
                - CalculateDET(ax, ay, ax * ax + ay * ay, cx, cy, cx * cx + cy * cy, dx, dy, dx * dx + dy * dy)
                + CalculateDET(ax, ay, ax * ax + ay * ay, bx, by, bx * bx + by * by, dx, dy, dx * dx + dy * dy);
        return det > 0;
    }

    template<typename T>
    bool IsInCircumcircle(const Point2d<T>& p, const Point2d<T>& v1, const Point2d<T>& v2, const Point2d<T>& v3) {
        return InCircle(v1.x, v1.y, v2.x, v2.y, v3.x, v3.y, p.x, p.y);
    }

    template<typename T>
    bool touchesSuperTriangle(const Triangle2d<T>& triangle) {
        return triangle.vertices[0] < 3 or triangle.vertices[1] < 3 or triangle.vertices[2] < 3;
    }

}

#endif //CONSTRAINTDELAUNAYTRIANGULATION_CDTUTILS_H
