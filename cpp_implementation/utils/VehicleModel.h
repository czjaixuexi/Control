/**
 * @file VehicleModel.h
 * @author czj
 * @brief
 * @version 0.1
 * @date 2023-05-12
 *
 * @copyright Copyright (c) 2023
 *
 */

#ifndef VehicleModel_H
#define VehicleModel_H
#include <iostream>
#include <vector>
#include <cmath>
#include <Eigen/Dense>


 // 车辆状态
class State {
public:
    double x;     // 质心x位置
    double y;     // 质心y位置
    double yaw;    // 航向角
    double v;     // 速度
    double l;    // 轴距
    double rear_x; //后轴x
    double rear_y;  //后轴y
    double front_x; //前轴x
    double front_y; //前轴y
public:
    State(double x, double y, double yaw, double v, double l);
};

// 车辆运动学模型
class VehicleModel {
public:
    double dt;  //采样时间
    State state; //车辆状态
public:
    VehicleModel(State& state, double dt);

    void updateState(double accel, double delta);

    State getState();

    //vector<MatrixXd> stateSpace(double ref_delta, double ref_yaw);

};


#endif //VehicleModel_H
