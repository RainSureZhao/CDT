#include <iostream>
#include <vector>
#include "geometry/point.h"
#include "geometry/edge.h"
#include "dt/delaunay2d.h"
#include <random>
#include <chrono>
#include <array>
#include <CGAL/Exact_predicates_inexact_constructions_kernel.h>
#include <CGAL/Surface_mesh.h>
#include <CGAL/draw_surface_mesh.h>
#include "utils/draw.h"
// #include "CDT.h"
// #include "utils/cdtUtils.h"

typedef CGAL::Exact_predicates_inexact_constructions_kernel K;
typedef K::Point_3                                          Point;
typedef CGAL::Surface_mesh<Point>                           Mesh;                                       Point;
int main()
{
    // CDT::Triangulation<double> cdt_generator;

    int n;
    std::cin >> n;
    std::default_random_engine eng(std::random_device{}());
    std::uniform_real_distribution<double> dist_w(0, 800);
    std::uniform_real_distribution<double> dist_h(0, 600);
    std::cout << "Generating " << n << " random points" << std::endl;
////
    std::vector<cdt::Point2d<double>> points;
    // std::vector<CDT::V2d<double>> ps;
//    cdt::Point2d<double> p{0, 0}, v1{-1, 0}, v2{0, 0}, v3{1, 0};
//    std::cout << cdt::IsInCircumcircle(p, v1, v2, v3) << std::endl;
    for(int i = 0; i < n; i ++) {
        // std::cin >> points[i];
        points.emplace_back(dist_w(eng), dist_h(eng));
        // ps.emplace_back(dist_w(eng), dist_h(eng));
    }
    cdt::Delaunay2d<double> triangulation;
    const auto start = std::chrono::high_resolution_clock::now();
    triangulation.insertVertices(points);
    triangulation.eraseSuperTriangle();
    std::cout << triangulation.checkResult() << std::endl;
    auto triangles = triangulation.getTriangles();
    // const std::vector<cdt::Triangle2d<double>> triangles = triangulation.triangulate(points);
    const auto end = std::chrono::high_resolution_clock::now();
    const std::chrono::duration<double> diff = end - start;
    std::cout << triangles.size() << " triangles generated in " << diff.count()
          << "s\n";
//    for(auto &triangle : triangles) {
//        std::cout << triangle << std::endl;
//    }
     auto mesh = cdt::drawMesh(triangles);
//    cdt_generator.insertVertices(ps);
//    cdt_generator.eraseSuperTriangle();
//    auto triangles = cdt_generator.triangles;
//    // for(auto &point : points) std::cin >> point;
////    for(auto &point : points) {
////        std::cout << point << std::endl;
////    }
////    cdt::Triangle2d<double> tri{{-100, 100}, {0, 200}, {100, 100}};
////    std::cout << tri.circumCircleContains({0, 0}) << std::endl;
//    cdt::Delaunay2d<double> triangulation;
//    const auto start = std::chrono::high_resolution_clock::now();
//    const std::vector<cdt::Triangle2d<double>> triangles = triangulation.triangulate(points);
//    const auto end = std::chrono::high_resolution_clock::now();
//    const std::chrono::duration<double> diff = end - start;
//
//    std::cout << triangles.size() << " triangles generated in " << diff.count()
//              << "s\n";
//
//    auto mesh = cdt::drawMesh(triangles);

//    Mesh mesh;
//    std::vector<std::array<Mesh::Vertex_index, 3>> triangles_indexs;
//    for(std::size_t i = 0; i < triangles.size(); i ++) {
//        auto a = triangles[i].a, b = triangles[i].b, c = triangles[i].c;
//        auto id1 = mesh.add_vertex(Point(a.x, a.y, 0));
//        auto id2 = mesh.add_vertex(Point(b.x, b.y, 0));
//        auto id3 = mesh.add_vertex(Point(c.x, c.y, 0));
//        triangles_indexs.push_back({id1, id2, id3});
//    }
//    for(const auto& tri : triangles_indexs) {
//        mesh.add_face(tri[0], tri[1], tri[2]);
//    }
//    CGAL::draw(mesh);
    return 0;
}
