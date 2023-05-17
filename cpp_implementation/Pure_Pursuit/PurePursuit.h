/**
 * @file PurePursuit.h
 * @author czj
 * @brief
 * @version 0.1
 * @date 2023-05-11
 *
 * @copyright Copyright (c) 2023
 *
 */

#ifndef PP_H
#define PP_H
#include <iostream>
#include <vector>
#include <cmath>
#include <algorithm>
#include "../utils/VehicleModel.h"
#include "../utils/ReferenceLine.h"
class PurePursuit {
public:
    int old_nearest_point_index = -1; // 上一个最近点的index

public:
    double calc_distance(State state, std::vector<ReferPoint> refer, int index);

    double search_target_index(State state, std::vector<ReferPoint> refer, double l_d);

    double pure_pursuit_steer_control(State state, std::vector<ReferPoint> refer, int min_index, double l_d, double L);
};



#endif //PP_H
