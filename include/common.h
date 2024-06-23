//
// Created by RainSure on 24-6-22.
//

#ifndef CONSTRAINTDELAUNAYTRIANGULATION_COMMON_H
#define CONSTRAINTDELAUNAYTRIANGULATION_COMMON_H

#include <numeric>
#include <limits>
#include <vector>
#include <array>
namespace cdt {
    using IndexSizeType = unsigned int;
    /// 三角形内的索引
    using Index = IndexSizeType;
    /// 顶点索引
    using VertexIndex = IndexSizeType;
    /// 三角形索引
    using TriangleIndex = IndexSizeType;
    /// 无效索引
    const IndexSizeType invalidIndex = static_cast<IndexSizeType>(std::numeric_limits<IndexSizeType>::max());
    /// 一个三角形没有合法邻居三角形索引
    const TriangleIndex noNeighbor(invalidIndex);
    /// 一个三角形没有合法顶点索引
    const VertexIndex noVertex(invalidIndex);
    /// 三角形索引数组
    using TriangleIndexVector = std::vector<TriangleIndex>;
    /// 三个顶点索引数组
    using VertexIndexArray3 = std::array<VertexIndex, 3>;
    /// 三个邻居节点索引数组
    using NeighborIndexArray3 = std::array<TriangleIndex, 3>;
    /// 三角形数组
//    template<typename T>
//    using TriangleVector = std::vector<Triangle2d<T>>;
}

#endif //CONSTRAINTDELAUNAYTRIANGULATION_COMMON_H
