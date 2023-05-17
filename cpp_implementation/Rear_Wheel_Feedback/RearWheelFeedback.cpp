/**
 * @file RearWheelFeedback.cpp
 * @author czj
 * @brief
 * @version 0.1
 * @date 2023-05-15
 *
 * @copyright Copyright (c) 2023
 *
 */

#include "RearWheelFeedback.h"

 /**
  * @brief 计算后轴与参考轨迹某点距离
  *
  * @param state 车辆状态
  * @param refer 参考轨迹
  * @param index 轨迹下标
  * @return double
  */
double RearWheelFeedback::calc_distance(State state, std::vector<ReferPoint> refer, int index)
{
    return std::sqrt(std::pow(state.rear_x - refer[index].x, 2) + std::pow(state.rear_y - refer[index].y, 2));
}


/**
 * @brief 寻找匹配点下标
 * @param state 车辆状态
 * @param refer 参考轨迹
 * @return double
 */
double RearWheelFeedback::search_target_index(State state, std::vector<ReferPoint> refer)
{
    std::vector<double> dists;
    double min_index;
    //std::cout << old_nearest_point_index << std::endl;
    // 寻找匹配点
    // 若是第一次搜寻，则遍历所有参考点寻找最小值下标
    if (old_nearest_point_index == -1) {
        for (int i = 0; i < refer.size();i++) {
            double dist = calc_distance(state, refer, i);
            dists.push_back(dist);
        }
        min_index = std::min_element(dists.begin(), dists.end()) - dists.begin(); //返回vector最小元素的下标
        old_nearest_point_index = min_index;
    }
    // 否则直接从上一个匹配点开始查找（找到第一个距离开始增大的点，类似梯度下降）
    else {
        min_index = old_nearest_point_index;
        for (int i = min_index; i < refer.size() - 1; i++) {
            double distance_this_index = calc_distance(state, refer, i);
            double distance_next_index = calc_distance(state, refer, i + 1);
            if (distance_this_index < distance_next_index) {
                break;
            }
            min_index = i + 1;
            //std::cout << old_nearest_point_index << std::endl;
        }
        old_nearest_point_index = min_index;
    }
    return min_index;
}

void RearWheelFeedback::compute_errors(State state, std::vector<ReferPoint> refer, double& e_y, double& theta_e, double& cur)
{
    int match_index = search_target_index(state, refer);     // 获取匹配点下标
    std::cout << "匹配点" << match_index << std::endl;
    if (match_index >= refer.size()) {
        match_index = refer.size() - 1;
    }
    cur = refer[match_index].cur;                           //匹配点曲率
    e_y = calc_distance(state, refer, match_index);         //计算横向误差

    // 将位置误差转换为前轮转角的时候：需要将路径上距离车辆最近的点从世界坐标系变换到车辆坐标系下，根据路径点在车辆坐标系下的横坐标的正负决定前轮转角的方向
    double match_point_y_in_vehicle_coordinate = -(refer[match_index].x - state.x) * std::sin(state.yaw) +
        (refer[match_index].y - state.y) * std::cos(state.yaw);
    // 车辆坐标系：X轴沿着车辆纵向，向前为正，Y沿着车辆横向，向左为正（从车头往前看的视角）
    if (match_point_y_in_vehicle_coordinate > 0)
    {
        e_y = -e_y;
    }
    else if (match_point_y_in_vehicle_coordinate < 0) {
        e_y = e_y;
    }
    theta_e = normalize_angle(state.yaw - refer[match_index].heading);    // 航向角误差
}


/**
 * @brief 角度归一化
 *
 * @param angle
 * @return double
 */
double RearWheelFeedback::normalize_angle(double angle) {
    while (angle > PI) {
        angle -= 2.0 * PI;
    }
    while (angle < -PI) {
        angle += 2.0 * PI;
    }
    return angle;
}

double RearWheelFeedback::rear_wheel_feedback_control(State state, std::vector<ReferPoint> refer, double K_psi, double K2)
{
    double e_y;
    double theta_e;
    double cur;
    // 计算误差
    compute_errors(state, refer, e_y, theta_e, cur);
    // 由李雅普诺夫方程求得角速度
    double psi_dot = state.v * cur * std::cos(theta_e) / (1.0 - cur * e_y) -
        K2 * state.v * e_y * std::sin(theta_e) / theta_e - K_psi * std::abs(state.v) * theta_e;

    if (theta_e == 0.0 || psi_dot == 0.0) {
        return 0.0;
    }
    double delta = std::atan2(state.l * psi_dot, state.v);
    std::cout << "控制量:" << delta << std::endl;
    return delta;
}




