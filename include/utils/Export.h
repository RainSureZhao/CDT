//
// Created by RainSure on 24-6-25.
//

#ifndef CONSTRAINTDELAUNAYTRIANGULATION_EXPORT_H
#define CONSTRAINTDELAUNAYTRIANGULATION_EXPORT_H

#include <fstream>
#include <string>
#include "geometry/point.h"

namespace cdt {
    // 导出二维数据点为.off格式
    template<typename T>
    void exportOff(const Point2d<T>& points, const std::string& filePath) {
        std::ofstream file(filePath);
        file << "OFF" << std::endl;
        file << points.size() << " 0 0" << std::endl;
        for(auto& point : points) {
            file << point.x << " " << point.y << " 0" << std::endl;
        }
        file.close();
    }
}


#endif //CONSTRAINTDELAUNAYTRIANGULATION_EXPORT_H
