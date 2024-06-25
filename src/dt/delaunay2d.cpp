//
// Created by RainSure on 24-6-19.
//

#include "dt/delaunay2d.h"
#include "utils/cdtUtils.h"
#include <unordered_set>
#include <unordered_map>

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
        _triangles.erase(std::remove_if(std::begin(_triangles), std::end(_triangles), [p1, p2, p3](auto& t) {
            return t.containsVertex(p1) || t.containsVertex(p2) || t.containsVertex(p3);
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

    template<typename T>
    TriangleIndex Delaunay2d<T>::addTriangle(const Triangle2d<T> &t) {
        if(m_dummyTris.empty()) {
            _triangles.push_back(t);
            return static_cast<TriangleIndex>(_triangles.size() - 1);
        }
        auto nxtDummy = m_dummyTris.back();
        m_dummyTris.pop_back();
        _triangles[nxtDummy] = t;
        return nxtDummy;
    }

    template<typename T>
    TriangleIndex Delaunay2d<T>::addTriangle() {
        if(m_dummyTris.empty()) {
            Triangle2d<T> dummy = {
                    {noVertex, noVertex, noVertex},
                    {noNeighbor, noNeighbor, noNeighbor}
            };
            _triangles.push_back(dummy);
            return static_cast<TriangleIndex>(_triangles.size() - 1);
        }
        auto nxtDummy = m_dummyTris.back();
        m_dummyTris.pop_back();
        return nxtDummy;
    }

    // find point in which triangle
    template<typename T>
    TriangleIndex Delaunay2d<T>::walkTriangles(VertexIndex startVertex, const Point2d<T> &point) const {
        // begin walk in search of triangle at pos
        TriangleIndex currentTriangle = m_vertTris[startVertex];
        bool found = false;
        SplitMix64RandGen prng;
        while(!found) {
            auto& t = _triangles[currentTriangle];
            found = true;
            // stochastic offset to randomize which edge we check first
            int offset(prng() % 3);
            for(auto i = 0; i < 3; i ++) {
                auto j = (i + offset) % 3;
                auto vStart = _vertices[t.vertices[j]];
                auto vEnd = _vertices[t.vertices[ccw(j)]];
                auto edgeCheck = LocatePointLine(point, vStart, vEnd);
                auto jN = t.neighbors[j];
                if(edgeCheck == PointLineLocation::Right && jN != noNeighbor) {
                    found = false;
                    currentTriangle = jN;
                    break;
                }
            }
        }
        return currentTriangle;
    }

    template<typename T>
    std::array<TriangleIndex, 2>
    Delaunay2d<T>::walkingSearchTrianglesAt(VertexIndex iV, cdt::VertexIndex startVertex) const {
        auto v = _vertices[iV];
        std::array<TriangleIndex, 2> out = {noNeighbor, noNeighbor};
        auto iT = walkTriangles(startVertex, v);
        auto triangle = _triangles[iT];
        auto location = LocatePointTriangle(v, _vertices[triangle.vertices[0]], _vertices[triangle.vertices[1]],
                                            _vertices[triangle.vertices[2]]);
        if(location == PointTriangleLocation::Outside) {
            throw std::runtime_error("WalkingSearchTrianglesAt: point outside triangle");
        }
        if (location == PointTriangleLocation::OnVertex) {
            throw std::runtime_error("WalkingSearchTrianglesAt: point duplicate");
        }
        out[0] = iT;
        if(IsOnEdge(location)) {
            out[1] = triangle.neighbors[EdgeNeighbor(location)];
        }
        return out;
    }

    template<typename T>
    std::array<TriangleIndex, 2> Delaunay2d<T>::trianglesAtBruteForce(const Point2d<T> &point) const {
        std::array<TriangleIndex, 2> out = {noNeighbor, noNeighbor};
        for(auto i = 0; i < _triangles.size(); i ++) {
            auto& t = _triangles[i];
            auto location = LocatePointTriangle(point, _vertices[t.vertices[0]], _vertices[t.vertices[1]],
                                                _vertices[t.vertices[2]]);
            if(location == PointTriangleLocation::Inside) {
                out[0] = i;
                break;
            }
            if(IsOnEdge(location)) {
                out[1] = t.neighbors[EdgeNeighbor(location)];
            }
        }
        return out;
    }

    template<typename T>
    void Delaunay2d<T>::insertVertex(cdt::VertexIndex iVert, cdt::VertexIndex walkStart) {
        auto trisAt = walkingSearchTrianglesAt(iVert, walkStart);
        auto triStack = trisAt[1] == noNeighbor ? insertVertexInsideTriangle(iVert, trisAt[0])
                : insertVertexOnEdge(iVert, trisAt[0], trisAt[1]);
        ensureDelaunayByEdgeFlip(iVert, triStack);
    }


    /* Insert point into triangle: split into 3 triangles:
     *  - create 2 new triangles
     *  - re-use old triangle for the 3rd
     *                      v3
     *                    / | \
     *                   /  |  \ <-- original triangle (t)
     *                  /   |   \
     *              n3 /    |    \ n2
     *                /newT2|newT1\
     *               /      v      \
     *              /    __/ \__    \
     *             /  __/       \__  \
     *            / _/      t'     \_ \
     *          v1 ___________________ v2
     *                     n1
     */
    template<typename T>
    std::stack<TriangleIndex>
    Delaunay2d<T>::insertVertexInsideTriangle(cdt::VertexIndex vertexIndex, cdt::TriangleIndex triangleIndex) {
        TriangleIndex  NewT1 = addTriangle();
        TriangleIndex  NewT2 = addTriangle();

        Triangle2d<T>& triangle = _triangles[triangleIndex];
        auto vertices = triangle.vertices;
        auto neighbors = triangle.neighbors;
        auto v1 = vertices[0], v2 = vertices[1], v3 = vertices[2];
        auto n1 = neighbors[0], n2 = neighbors[1], n3 = neighbors[2];

        // make two new triangles and convert current triangle to 3rd new
        // triangle
        _triangles[NewT1] = {{v2, v3, vertexIndex}, {n2, NewT2, triangleIndex}};
        _triangles[NewT2] = {{v3, v1, vertexIndex}, {n3, triangleIndex, NewT1}};
        triangle = {{v1, v2, vertexIndex}, {n1, NewT1, NewT2}};
        // adjust adjacent triangles
        setAdjacentTriangle(vertexIndex, triangleIndex);
        setAdjacentTriangle(v3, NewT1);
        // change triangle neighbor's neighbors to new triangles
        changeNeighbor(n2, triangleIndex, NewT1);
        changeNeighbor(n3, triangleIndex, NewT2);
        // return newly added triangles
        std::stack<TriangleIndex> newTriangles;
        newTriangles.push(triangleIndex);
        newTriangles.push(NewT1);
        newTriangles.push(NewT2);
        return newTriangles;
    }

    /* Inserting a point on the edge between two triangles
     *    T1 (top)        v1
     *                   /|\
     *              n1 /  |  \ n4
     *               /    |    \
     *             /  T1' | Tnew1\
     *           v2-------v-------v4
     *             \  T2' | Tnew2/
     *               \    |    /
     *              n2 \  |  / n3
     *                   \|/
     *   T2 (bottom)      v3
     */
    template<typename T>
    std::stack<TriangleIndex>
    Delaunay2d<T>::insertVertexOnEdge(cdt::VertexIndex vertexIndex, cdt::TriangleIndex iT1, cdt::TriangleIndex iT2) {
        TriangleIndex NewT1 = addTriangle();
        TriangleIndex NewT2 = addTriangle();

        Triangle2d<T>& triangle1 = _triangles[iT1];
        Triangle2d<T>& triangle2 = _triangles[iT2];
        int i = opposedVertexIndex(triangle1.neighbors, iT2);
        VertexIndex v1 = triangle1.vertices[i];
        VertexIndex v2 = triangle1.vertices[ccw(i)];
        TriangleIndex n1 = triangle1.neighbors[i];
        TriangleIndex n4 = triangle1.neighbors[cw(i)];
        i = opposedVertexIndex(triangle2.neighbors, iT1);
        VertexIndex v3 = triangle2.vertices[i];
        VertexIndex v4 = triangle2.vertices[ccw(i)];
        TriangleIndex n3 = triangle2.neighbors[i];
        TriangleIndex n2 = triangle2.neighbors[cw(i)];
        // add new triangles and change existing ones
        triangle1 = {{vertexIndex, v1, v2}, {NewT1, n1, iT2}};
        triangle2 = {{vertexIndex, v2, v3}, {iT1, n2, NewT2}};
        _triangles[NewT1] = {{vertexIndex, v4, v1}, {NewT1, n4, iT1}};
        _triangles[NewT2] = {{vertexIndex, v3, v4}, {iT2, n3, NewT1}};
        // adjust adjacent triangles
        setAdjacentTriangle(vertexIndex, iT1);
        setAdjacentTriangle(v4, NewT1);
        // adjust neighboring triangles and vertices
        changeNeighbor(n4, iT1, NewT1);
        changeNeighbor(n3, iT2, NewT2);
        // return newly added triangles
        std::stack<TriangleIndex> newTriangles;
        newTriangles.push(iT1);
        newTriangles.push(NewT2);
        newTriangles.push(iT2);
        newTriangles.push(NewT1);
        return newTriangles;
    }

    template<typename T>
    void Delaunay2d<T>::ensureDelaunayByEdgeFlip(cdt::VertexIndex iV, std::stack<TriangleIndex> &triangleStack) {
        TriangleIndex  iTopo, n1, n2, n3, n4;
        VertexIndex iV2, iV3, iV4;
        while(!triangleStack.empty()) {
            TriangleIndex iT = triangleStack.top();
            triangleStack.pop();
            edgeFlipInfo(iT, iV, iTopo, iV2, iV3, iV4, n1, n2, n3, n4);
            if(iTopo != noNeighbor && isFlipNeeded(iV, iV2, iV3, iV4)) {
                flipEdge(iT, iTopo, iV, iV2, iV3, iV4, n1, n2, n3, n4);
                triangleStack.push(iT);
                triangleStack.push(iTopo);
            }
        }
    }

    /*
     *                       v4         original edge: (v2, v4)
     *                      /|\   flip-candidate edge: (v1,  v3)
     *                    /  |  \
     *              n3  /    |    \  n4
     *                /      |      \
     * new vertex--> v1    T | Topo  v3
     *                \      |      /
     *              n1  \    |    /  n2
     *                    \  |  /
     *                      \|/
     *                       v2
     */
    template<typename T>
    void Delaunay2d<T>::flipEdge(cdt::TriangleIndex iT, cdt::TriangleIndex iTopo) {
        Triangle2d<T>& t = _triangles[iT];
        Triangle2d<T>& tOpo = _triangles[iTopo];
        std::array<TriangleIndex, 3>& triNs = t.neighbors;
        std::array<TriangleIndex, 3>& triOpoNs = tOpo.neighbors;
        std::array<VertexIndex, 3>& triVs = t.vertices;
        std::array<VertexIndex, 3>& triOpoVs = tOpo.vertices;
        // find vertices and neighbors
        Index i = opposedVertexIndex(t.neighbors, iTopo);
        VertexIndex v1 = triVs[i];
        VertexIndex v2 = triVs[ccw(i)];
        TriangleIndex n1 = triNs[i];
        TriangleIndex n3 = triNs[cw(i)];
        i = opposedVertexIndex(tOpo.neighbors, iT);
        VertexIndex v3 = triOpoVs[i];
        VertexIndex v4 = triOpoVs[ccw(i)];
        TriangleIndex n4 = triOpoNs[i];
        TriangleIndex n2 = triOpoNs[cw(i)];
        // change vertices and neighbors
        t = {{v4, v1, v3}, {n3, iTopo, n4}};
        tOpo = {{v2, v3, v1}, {n2, iT, n1}};
        // adjust neighboring triangles and vertices
        changeNeighbor(n1, iT, iTopo);
        changeNeighbor(n4, iTopo, iT);
        // only adjust adjacent triangles if triangulation is not finalized:
        // can happen when called from outside on an already finalized
        // triangulation
        if(!isFinalized())
        {
            setAdjacentTriangle(v4, iT);
            setAdjacentTriangle(v2, iTopo);
        }
    }

    /*
     *                       v4         original edge: (v1, v3)
     *                      /|\   flip-candidate edge: (v,  v2)
     *                    /  |  \
     *              n3  /    |    \  n4
     *                /      |      \
     * new vertex--> v1    T | Topo  v3
     *                \      |      /
     *              n1  \    |    /  n2
     *                    \  |  /
     *                      \|/
     *                       v2
     */
    template<typename T>
    void Delaunay2d<T>::flipEdge(cdt::TriangleIndex iT, cdt::TriangleIndex iTopo, cdt::VertexIndex v1,
                                 cdt::VertexIndex v2, cdt::VertexIndex v3, cdt::VertexIndex v4, cdt::TriangleIndex n1,
                                 cdt::TriangleIndex n2, cdt::TriangleIndex n3, cdt::TriangleIndex n4) {
        // change vertices and neighbors
        _triangles[iT] = {{v4, v1, v3}, {n3, iTopo, n4}};
        _triangles[iTopo] = {{v2, v3, v1}, {n2, iT, n1}};
        // adjust neighboring triangles and vertices
        changeNeighbor(n1, iT, iTopo);
        changeNeighbor(n4, iTopo, iT);
        // only adjust adjacent triangles if triangulation is not finalized:
        // can happen when called from outside on an already finalized
        // triangulation
        if(!isFinalized())
        {
            setAdjacentTriangle(v4, iT);
            setAdjacentTriangle(v2, iTopo);
        }
    }

    /*!
     * Handles super-triangle vertices.
     * Super-tri points are not infinitely far and influence the input points
     * Three cases are possible:
     *  1.  If one of the opposed vertices is super-tri: no flip needed
     *  2.  One of the shared vertices is super-tri:
     *      check if on point is same side of line formed by non-super-tri
     * vertices as the non-super-tri shared vertex
     *  3.  None of the vertices are super-tri: normal circumcircle test
     */
    /*
     *                       v4         original edge: (v2, v4)
     *                      /|\   flip-candidate edge: (v1, v3)
     *                    /  |  \
     *                  /    |    \
     *                /      |      \
     * new vertex--> v1      |       v3
     *                \      |      /
     *                  \    |    /
     *                    \  |  /
     *                      \|/
     *                       v2
     */
    template<typename T>
    bool Delaunay2d<T>::isFlipNeeded(cdt::VertexIndex iV1, cdt::VertexIndex iV2, cdt::VertexIndex iV3,
                                     cdt::VertexIndex iV4) const {
        auto& v1 = _vertices[iV1];
        auto& v2 = _vertices[iV2];
        auto& v3 = _vertices[iV3];
        auto& v4 = _vertices[iV4];
        // If flip-candidate edge touches super-triangle in-circumference
        // test has to be replaced with orient2d test against the line
        // formed by two non-artificial vertices (that don't belong to
        // super-triangle)
        if(iV1 < 3) { // flip-candidate edge touches super-triangle
            // does original edge also touch super-triangle ?
            if(iV2 < 3) {
                return LocatePointLine(v2, v3, v4) == LocatePointLine(v1, v3, v4);
            }
            if(iV4 < 3) {
                return LocatePointLine(v4, v2, v3) == LocatePointLine(v1, v2, v3);
            }
            return false; // original edge does not touch super-triangle
        }
        if(iV3 < 3) { // flip-candidate edge touches super-triangle
            // does original edge also touch super-triangle ?
            if(iV2 < 3) {
                return LocatePointLine(v2, v1, v4) == LocatePointLine(v3, v1, v4);
            }
            if(iV4 < 3) {
                return LocatePointLine(v4, v2, v1) == LocatePointLine(v3, v2, v1);
            }
            return false; // original edge does not touch super-triangle
        }
        // flip-candidate edge does not touch super-triangle
        if(iV2 < 3) {
            return LocatePointLine(v2, v3, v4) == LocatePointLine(v1, v3, v4);
        }
        if(iV4 < 3) {
            return LocatePointLine(v4, v2, v3) == LocatePointLine(v1 ,v2, v3);
        }
        return IsInCircumcircle(v1, v2, v3, v4);
    }

    /*
     *                       v4         original edge: (v1, v3)
     *                      /|\   flip-candidate edge: (v,  v2)
     *                    /  |  \
     *              n3  /    |    \  n4
     *                /      |      \
     * new vertex--> v1    T | Topo  v3
     *                \      |      /
     *              n1  \    |    /  n2
     *                    \  |  /
     *                      \|/
     *                       v2
     */
    template<typename T>
    void Delaunay2d<T>::edgeFlipInfo(cdt::TriangleIndex iT, cdt::VertexIndex iV1, cdt::TriangleIndex &iTopo,
                                     cdt::VertexIndex &iV2, cdt::VertexIndex &iV3, cdt::VertexIndex &iV4,
                                     cdt::TriangleIndex &n1, cdt::TriangleIndex &n2, cdt::TriangleIndex &n3,
                                     cdt::TriangleIndex &n4) {
        /*     v[2]
               / \
          n[2]/   \n[1]
             /_____\
        v[0]  n[0]  v[1]  */
        auto& t = _triangles[iT];
        if(t.vertices[0] == iV1)
        {
            iV2 = t.vertices[1];
            iV4 = t.vertices[2];
            n1 = t.neighbors[0];
            n3 = t.neighbors[2];
            iTopo = t.neighbors[1];
        }
        else if(t.vertices[1] == iV1)
        {
            iV2 = t.vertices[2];
            iV4 = t.vertices[0];
            n1 = t.neighbors[1];
            n3 = t.neighbors[0];
            iTopo = t.neighbors[2];
        }
        else
        {
            iV2 = t.vertices[0];
            iV4 = t.vertices[1];
            n1 = t.neighbors[2];
            n3 = t.neighbors[1];
            iTopo = t.neighbors[0];
        }
        if(iTopo == noNeighbor)
            return;
        auto& tOpo = _triangles[iTopo];
        if(tOpo.neighbors[0] == iT)
        {
            iV3 = tOpo.vertices[2];
            n2 = tOpo.neighbors[1];
            n4 = tOpo.neighbors[2];
        }
        else if(tOpo.neighbors[1] == iT)
        {
            iV3 = tOpo.vertices[0];
            n2 = tOpo.neighbors[2];
            n4 = tOpo.neighbors[0];
        }
        else
        {
            iV3 = tOpo.vertices[1];
            n2 = tOpo.neighbors[0];
            n4 = tOpo.neighbors[1];
        }
    }

    template<typename T>
    void Delaunay2d<T>::changeNeighbor(cdt::TriangleIndex iT, cdt::TriangleIndex oldNeighbor,
                                       cdt::TriangleIndex newNeighbor) {
        if(iT == noNeighbor) {
            return;
        }
        auto& neighbors = _triangles[iT].neighbors;
        assert(neighbors[0] == oldNeighbor or neighbors[1] == oldNeighbor or neighbors[2] == oldNeighbor);
        if(neighbors[0] == oldNeighbor) {
            neighbors[0] = newNeighbor;
        } else if(neighbors[1] == oldNeighbor) {
            neighbors[1] = newNeighbor;
        } else {
            neighbors[2] = newNeighbor;
        }
    }

    template<typename T>
    bool Delaunay2d<T>::isFinalized() const {
        return m_vertTris.empty() && !_vertices.empty();
    }

    template<typename T>
    void Delaunay2d<T>::setAdjacentTriangle(cdt::VertexIndex vertex, cdt::TriangleIndex triangle) {
        assert(triangle != noNeighbor);
        m_vertTris[vertex] = triangle;
        assert(_triangles[triangle].vertices[0] == vertex or _triangles[triangle].vertices[1] == vertex
        or _triangles[triangle].vertices[2] == vertex);
    }

    template<typename T>
    void Delaunay2d<T>::addNewVertex(const Point2d<T> &point, cdt::TriangleIndex iT) {
        _vertices.push_back(point);
        m_vertTris.push_back(iT);
    }

    template<typename T>
    void Delaunay2d<T>::addSuperTriangle(const std::vector<Point2d<T>>& points) {
        auto minX = points[0].x, maxX = points[0].x, minY = points[0].y, maxY = points[0].y;
        for(std::size_t i = 0; i < points.size(); i ++) {
            minX = std::min(minX, points[i].x);
            maxX = std::max(maxX, points[i].x);
            minY = std::min(minY, points[i].y);
            maxY = std::max(maxY, points[i].y);
        }

        auto dx = maxX - minX;
        auto dy = maxY - minY;
        auto deltaMax = std::max(dx, dy);
        auto midX = (minX + maxX) / 2;
        auto midY = (minY + maxY) / 2;

        Point2d<T> p1{midX - 20 * deltaMax, midY - deltaMax}; // 左下角
        Point2d<T> p2{midX, midY + 20 * deltaMax};            // 上顶点
        Point2d<T> p3{midX + 20 * deltaMax, midY - deltaMax}; // 右下角

        addNewVertex(p1, TriangleIndex(0));
        addNewVertex(p2, TriangleIndex(0));
        addNewVertex(p3, TriangleIndex(0));

        Triangle2d<T> superTriangle = {
                {0, 1, 2},
                {noNeighbor, noNeighbor, noNeighbor}
        };
        addTriangle(superTriangle);
    }

    template<typename T>
    void Delaunay2d<T>::insertVertices(const std::vector<Point2d<T>> &points) {
        if(_vertices.empty()) {
            addSuperTriangle(points);
        }

        auto  nExistingVerts = static_cast<VertexIndex>(_vertices.size());
        auto nVerts = static_cast<VertexIndex>(nExistingVerts + points.size());

        _triangles.reserve(_triangles.size() + 2 * nVerts);
        _vertices.reserve(nVerts);
        m_vertTris.reserve(nVerts);
        for(auto &point : points) {
            addNewVertex(point, noNeighbor);
        }
        for(auto i = nExistingVerts; i < nVerts; i ++) {
            insertVertex(i, 0);
        }
        for(auto &triangle : _triangles) {
            triangle.a = _vertices[triangle.vertices[0]];
            triangle.b = _vertices[triangle.vertices[1]];
            triangle.c = _vertices[triangle.vertices[2]];
        }
    }

    template<typename T>
    void Delaunay2d<T>::removeTriangles(const std::unordered_set<TriangleIndex> &removedTriangles) {
        if(removedTriangles.empty()) return;
        std::unordered_map<TriangleIndex, TriangleIndex> triangleMap;
        for(auto iT = 0, iTNew = 0; iT < _triangles.size(); iT ++) {
            if(removedTriangles.count(iT)) continue;
            triangleMap[iT] = iTNew;
            _triangles[iTNew] = _triangles[iT];
            iTNew ++;
        }
        _triangles.erase(_triangles.end() - removedTriangles.size(), _triangles.end());
        // adjust triangles' neighbors
        for(auto& triangle : _triangles) {
            for(auto i = 0; i < 3; i ++) {
                if(removedTriangles.count(triangle.neighbors[i])) {
                    triangle.neighbors[i] = noNeighbor;
                } else {
                    triangle.neighbors[i] = triangleMap[triangle.neighbors[i]];
                }
            }
        }
    }

    template<typename T>
    void Delaunay2d<T>::finalizeTriangulation(const std::unordered_set<TriangleIndex> &removedTriangles) {
        _vertices.erase(std::begin(_vertices), std::begin(_vertices) + 3);
        removeTriangles(removedTriangles);
        for(auto &triangle : _triangles) {
            for(auto &vertex : triangle.vertices) {
                vertex -= 3;
            }
        }
    }

    template<typename T>
    void Delaunay2d<T>::eraseSuperTriangle() {
        std::unordered_set<TriangleIndex> toErase;
        for(auto i = 0; i < _triangles.size(); i ++) {
            if(touchesSuperTriangle(_triangles[i])) {
                toErase.insert(i);
            }
        }
        finalizeTriangulation(toErase);
    }

    template<typename T>
    bool Delaunay2d<T>::checkResult() const {
        // 判断每个三角形的外接圆是否还含有其它点
        for(auto& tri : _triangles) {
            for(std::size_t i = 0; i < _vertices.size(); i ++) {
                if(i == tri.vertices[0] || i == tri.vertices[1] || i == tri.vertices[2]) {
                    continue;
                }
                if(IsInCircumcircle(_vertices[i], _vertices[tri.vertices[0]], _vertices[tri.vertices[1]], _vertices[tri.vertices[2]]))  {
                    return false;
                }
            }
        }
        return true;
    }

    template class Delaunay2d<float>;
    template class Delaunay2d<double>;

} // cdt