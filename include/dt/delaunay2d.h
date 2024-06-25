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
#include <stack>
#include <unordered_set>
#include "common.h"

namespace cdt {

    /// SplitMix64  pseudo-random number generator
    struct SplitMix64RandGen
    {
        typedef unsigned long long uint64;
        uint64 m_state;
        explicit SplitMix64RandGen(uint64 state)
                : m_state(state)
        {}
        explicit SplitMix64RandGen()
                : m_state(0)
        {}
        uint64 operator()()
        {
            uint64 z = (m_state += 0x9e3779b97f4a7c15);
            z = (z ^ (z >> 30)) * 0xbf58476d1ce4e5b9;
            z = (z ^ (z >> 27)) * 0x94d049bb133111eb;
            return z ^ (z >> 31);
        }
    };

    template<typename T>
    class Delaunay2d {
    public:
        Delaunay2d() = default;
        Delaunay2d(const Delaunay2d& ) = delete;
        Delaunay2d(Delaunay2d&& ) = delete;
        Delaunay2d& operator=(const Delaunay2d&) = delete;
        Delaunay2d& operator=(Delaunay2d&&) = delete;

        TriangleIndex addTriangle(const Triangle2d<T>& t);

        TriangleIndex addTriangle();

        TriangleIndex walkTriangles(VertexIndex startVertex, const Point2d<T>& point) const;
        [[nodiscard]] std::array<TriangleIndex, 2> walkingSearchTrianglesAt(VertexIndex iV, VertexIndex startVertex) const;

        std::array<TriangleIndex, 2> trianglesAtBruteForce(const Point2d<T>& point) const;

        void insertVertex(VertexIndex iVert, VertexIndex walkStart);

        std::stack<TriangleIndex> insertVertexInsideTriangle(VertexIndex vertexIndex, TriangleIndex triangleIndex);

        std::stack<TriangleIndex> insertVertexOnEdge(VertexIndex vertexIndex, TriangleIndex iT1, TriangleIndex iT2);

        bool isFlipNeeded(VertexIndex iV1, VertexIndex iV2, VertexIndex iV3, VertexIndex iV4) const;

        void ensureDelaunayByEdgeFlip(VertexIndex iV, std::stack<TriangleIndex>& triangleStack);

        void edgeFlipInfo(TriangleIndex iT, VertexIndex iV1, TriangleIndex& iTopo, VertexIndex& iV2, VertexIndex& iV3, VertexIndex& iV4, TriangleIndex& n1, TriangleIndex& n2, TriangleIndex& n3, TriangleIndex& n4);

        void flipEdge(TriangleIndex iT, TriangleIndex iTopo, VertexIndex v1, VertexIndex v2, VertexIndex v3, VertexIndex v4, TriangleIndex n1, TriangleIndex n2, TriangleIndex n3, TriangleIndex n4);

        void flipEdge(TriangleIndex iT, TriangleIndex iTopo);

        void changeNeighbor(TriangleIndex iT, TriangleIndex oldNeighbor, TriangleIndex newNeighbor);

        bool isFinalized() const;

        void setAdjacentTriangle(VertexIndex vertex, TriangleIndex triangle);

        void addNewVertex(const Point2d<T>& point, TriangleIndex iT);

        void addSuperTriangle(const std::vector<Point2d<T>>& points);

        void insertVertices(const std::vector<Point2d<T>>& points);

        void removeTriangles(const std::unordered_set<TriangleIndex>& removedTriangles);

        void eraseSuperTriangle();

        void finalizeTriangulation(const std::unordered_set<TriangleIndex>& removedTriangles);

        bool checkResult() const;

        const std::vector<Triangle2d<T>>& triangulate(std::vector<Point2d<T>>& points);
        const std::vector<Triangle2d<T>>& getTriangles() const;
        const std::vector<Edge2d<T>>& getEdges() const;
        const std::vector<Point2d<T>>& getVertices() const;

    private:
        std::vector<TriangleIndex> m_dummyTris;
        std::vector<Triangle2d<T>> _triangles;
        std::vector<Edge2d<T>> _edges;
        std::vector<Point2d<T>> _vertices;
        TriangleIndexVector m_vertTris; // one triangle adjacent to each vertex
    };

} // cdt

#endif //CDT_DELAUNAY2D_H
