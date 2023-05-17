/**
 * @file VehicleModel.cpp
 * @author czj
 * @brief
 * @version 0.1
 * @date 2023-05-13
 *
 * @copyright Copyright (c) 2023
 *
 */

#include "VehicleModel.h"

using std::vector;

State::State(double x, double y, double yaw, double v, double l) {
    this->x = x;
    this->y = y;
    this->yaw = yaw;
    this->v = v;
    this->l = l;
    this->rear_x = x - std::cos(yaw);
    this->rear_y = y - std::sin(yaw);
    this->front_x = x + std::cos(yaw);
    this->front_y = y + std::sin(yaw);
}


VehicleModel::VehicleModel(State& state, double dt) : state(state), dt(dt) {}

/**
 * @brief 更新车辆位置
 * @param accel 加速度
 * @param delta 转向角
 */
void VehicleModel::updateState(double accel, double delta) {
    state.x = state.x + state.v * std::cos(state.yaw) * dt;
    state.y = state.y + state.v * std::sin(state.yaw) * dt;
    state.yaw = state.yaw + state.v / state.l * std::tan(delta) * dt;
    state.v = state.v + accel * dt;
    // 默认车辆前后轴距比例为1:1
    state.rear_x = state.x - state.l / 2 * std::cos(state.yaw);
    state.rear_y = state.y - state.l / 2 * std::sin(state.yaw);
    state.front_x = state.x + state.l / 2 * std::cos(state.yaw);
    state.front_y = state.y + state.l / 2 * std::sin(state.yaw);
}

/**
 * @brief 获取车辆状态
 * @return
 */
State VehicleModel::getState() {
    return { state };
}


