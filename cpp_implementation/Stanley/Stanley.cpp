/**
 * @file Stanley.cpp
 * @author czj
 * @brief
 * @version 0.1
 * @date 2023-05-13
 *
 * @copyright Copyright (c) 2023
 *
 */
#include "Stanley.h"
using std::vector;


/**
 * @brief 计算车辆前轴与参考点的距离
 *
 * @param state 车辆状态
 * @param refer 参考轨迹
 * @param index 参考点下标
 * @return double
 */
double Stanley::calc_distance(State state, std::vector<ReferPoint> refer, int index)
{
    return std::sqrt(std::pow(state.front_x - refer[index].x, 2) + std::pow(state.front_y - refer[index].y, 2));
}


/**
 * @brief 寻找匹配点下标
 *
 * @param state 车辆状态
 * @param refer 参考轨迹
 * @return double
 */
double Stanley::search_target_index(State state, std::vector<ReferPoint> refer)
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


/**
 * @brief 角度归一化
 * @param angle
 * @return
 */
double Stanley::normalize_angle(double angle) {
    while (angle > PI) {
        angle -= 2.0 * PI;
    }
    while (angle < -PI) {
        angle += 2.0 * PI;
    }
    return angle;
}

/**
 * @brief 计算误差
 *
 * @param state 车辆当前状态
 * @param refer 参考轨迹
 * @param e_y 横向误差
 * @param e_theta 航向角误差
 */

void Stanley::compute_errors(State state, std::vector<ReferPoint> refer, double& e_y, double& theta_e) {

    int match_index = search_target_index(state, refer);     // 获取匹配点下标
    std::cout << "匹配点" << match_index << std::endl;
    if (match_index >= refer.size()) {
        match_index = refer.size() - 1;
    }
    e_y = calc_distance(state, refer, match_index);         //计算横向误差

    // 将位置误差转换为前轮转角的时候：需要将路径上距离车辆最近的点从世界坐标系变换到车辆坐标系下，根据路径点在车辆坐标系下的横坐标的正负决定前轮转角的方向
    double match_point_y_in_vehicle_coordinate = -(refer[match_index].x - state.x) * std::sin(state.yaw) + (refer[match_index].y - state.y) * std::cos(state.yaw);
    // 车辆坐标系：X轴沿着车辆纵向，向前为正，Y沿着车辆横向，向左为正（从车头往前看的视角）
    if (match_point_y_in_vehicle_coordinate > 0)
    {
        e_y = e_y;
        std::cout << "目标位于车辆左侧:" << e_y << std::endl;
    }
    else if (match_point_y_in_vehicle_coordinate < 0) {
        e_y = -e_y;
        std::cout << "目标位于车辆右侧:" << e_y << std::endl;
    }

    theta_e = (refer[match_index].heading - state.yaw);    // 航向角误差
}


/**
 * @brief stanley控制
 *
 * @param state 车辆状态
 * @param refer 参考轨迹
 * @param k 增益系数
 * @return double
 */
double Stanley::stanley_control(State state, std::vector<ReferPoint> refer, double k) {
    double e_y; //横向误差
    double theta_e; //航向角误差
    compute_errors(state, refer, e_y, theta_e); //计算误差
    std::cout << "航向角误差" << theta_e << std::endl;
    // 计算转角控制量
    double delta_e = atan2(k * e_y, state.v);
    std::cout << "横向误差控制转角" << delta_e << std::endl;
    double delta = normalize_angle(delta_e + theta_e);
    std::cout << "控制量" << delta << std::endl;
    return delta;
}


