/**
 * @file Stanley.h
 * @author czj
 * @brief
 * @version 0.1
 * @date 2023-05-13
 *
 * @copyright Copyright (c) 2023
 *
 */

#ifndef STANLEY_H
#define STANLEY_H
#include <iostream>
#include <vector>
#include <cmath>
#include <algorithm>
#include "../utils/VehicleModel.h"
#include "../utils/ReferenceLine.h"

#define PI 3.1415926

class Stanley {
private:
    int old_nearest_point_index = -1; // 上一个最近点的index
public:

    double calc_distance(State state, std::vector<ReferPoint> refer, int index);

    double search_target_index(State state, std::vector<ReferPoint> refer);

    void compute_errors(State state, std::vector<ReferPoint> refer, double& e_y, double& theta_e);

    double normalize_angle(double angle);

    double stanley_control(State state, std::vector<ReferPoint> refer, double k);

};


#endif //STANLEY_H
