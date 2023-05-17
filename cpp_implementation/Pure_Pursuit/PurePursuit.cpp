/**
 * @file PurePursuit.cpp
 * @author czj
 * @brief
 * @version 0.1
 * @date 2023-05-11
 *
 * @copyright Copyright (c) 2023
 *
 */

#include "PurePursuit.h"

/**
 * @brief 计算车辆后轴与参考点的距离
 *
 * @param state 车辆状态
 * @param refer 参考轨迹
 * @param index 参考点下标
 * @return double
 */
double PurePursuit::calc_distance(State state, std::vector<ReferPoint> refer, int index)
{
    return std::sqrt(std::pow(state.rear_x - refer[index].x, 2) + std::pow(state.rear_y - refer[index].y, 2));
}


/**
 * @brief 寻找预瞄点下标
 *
 * @param state 车辆状态
 * @param referx 参考轨迹
 * @param l_d 预瞄距离
 * @return double
 */
double PurePursuit::search_target_index(State state, std::vector<ReferPoint> refer, double l_d)
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
            std::cout << old_nearest_point_index << std::endl;
        }
        old_nearest_point_index = min_index;
    }
    // 从匹配点开始寻找预瞄点
    while (l_d > calc_distance(state, refer, min_index) && min_index < refer.size()) {
        min_index += 1;
    }
    return min_index;
}


/**
 * @brief PP控制输出转角
 *
 * @param state 车辆状态
 * @param refer 参考轨迹
 * @param min_index 预瞄点下标
 * @param l_d 预瞄距离
 * @param L 轴距
 * @return double
 */
double PurePursuit::pure_pursuit_steer_control(State state, std::vector<ReferPoint> refer, int min_index, double l_d, double L)
{
    double alpha = std::atan2(refer[min_index].y - state.rear_y, refer[min_index].x - state.rear_x) - state.yaw;
    double delta = std::atan2(2 * L * std::sin(alpha), l_d);
    return delta;
}





