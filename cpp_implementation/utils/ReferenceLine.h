/**
 * @file ReferenceLine.h
 * @author czj
 * @brief
 * @version 0.1
 * @date 2023-05-14
 *
 * @copyright Copyright (c) 2023
 *
 */

#ifndef REFERENCELINE_H
#define REFERENCELINE_H
#include <iostream>
#include <vector>
#include <cmath>
#include <algorithm>
#define PI 3.1415926

 // 参考点
class ReferPoint {
public:
    double x;
    double y;
    double heading; // 航向角
    double cur;   //曲率
};

// 参考轨迹
class ReferenceLine {
public:
    std::vector<ReferPoint> refer;
public:
    ReferenceLine();
};


#endif //REFERENCELINE_H
