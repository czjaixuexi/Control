/**
 * @file RearWheelFeedback.h
 * @author czj
 * @brief
 * @version 0.1
 * @date 2023-05-14
 *
 * @copyright Copyright (c) 2023
 *
 */

#ifndef RW_H
#define RW_H
#include <iostream>
#include <vector>
#include <cmath>
#include <algorithm>
#include "../utils/ReferenceLine.h"
#include "../utils/VehicleModel.h"

#define PI 3.1415926

class RearWheelFeedback {
private:
    int old_nearest_point_index = -1; // 上一个最近点的index
public:
    double calc_distance(State state, std::vector<ReferPoint> refer, int index);

    double search_target_index(State state, std::vector<ReferPoint> refer);

    void compute_errors(State state, std::vector<ReferPoint> refer, double& e_y, double& theta_e, double& cur);

    double normalize_angle(double angle);

    double rear_wheel_feedback_control(State state, std::vector<ReferPoint> refer, double K_psi, double K2);
};


#endif //RW_H
