/**
 * @file main.cpp
 * @author czj
 * @brief
 * @version 0.1
 * @date 2023-05-13
 *
 * @copyright Copyright (c) 2023
 *
 */

#include "PurePursuit.h"
#include "../utils/matplotlibcpp.h"

namespace plt = matplotlibcpp;

#define PI 3.1415926

using std::vector;
using std::sin;
using std::cos;

int main() {
    double x = 0.0, y = 0.0, yaw = 0.0, v = 30.0 / 3.6, l = 2.9, dt = 0.1;
    // 车辆状态
    State state(x, y, yaw, v, l);
    // 车辆模型
    VehicleModel vehicle(state, dt);
    double lam = 0.31; // 前视距离系数
    double c = 1.5; // 前视距离
    // 生成参考轨迹
    ReferenceLine refer_line;
    //后面画图用
    vector<double> refer_x(refer_line.refer.size());
    vector<double> refer_y(refer_line.refer.size());
    for (int i = 0;i < refer_line.refer.size();i++) {
        refer_x[i] = refer_line.refer[i].x;
        refer_y[i] = refer_line.refer[i].y;
    }

    // 车辆运动轨迹
    vector<double> trajectory_x, trajectory_y;
    PurePursuit pp;
    while (true) {
        plt::clf();
        //更新预瞄距离
        double l_d = lam * vehicle.state.v + c;
        //寻找预瞄点下标
        double min_index = pp.search_target_index(vehicle.state, refer_line.refer, l_d);
        //std::cout << min_index << std::endl;
        //计算转角控制量
        double delta = pp.pure_pursuit_steer_control(vehicle.state, refer_line.refer, min_index, l_d, l);
        //std::cout << delta << std::endl;
        //控制车辆，更新状态
        vehicle.updateState(0, delta);

        // 记录车辆运行轨迹，用于绘图
        trajectory_x.push_back(vehicle.state.x);
        trajectory_y.push_back(vehicle.state.y);

        //std::cout << vehicle.state.x << " " << vehicle.state.y << std::endl;

        //画图
        plt::plot(refer_x, refer_y, "b--");
        plt::plot(trajectory_x, trajectory_y, "r");
        plt::grid(true);
        plt::ylim(-8, 8);
        plt::pause(0.01);
        //控制结束跳出
        if (pp.calc_distance(vehicle.state, refer_line.refer, refer_line.refer.size() - 1) < 0.5) {
            std::cout << "Control completion" << std::endl;
            break;
        }
    }
    // 保存图像
    const char* filename = "./pure_pursuit_demo.png";
    std::cout << "Saving result to " << filename << std::endl;
    plt::save(filename);
    plt::show();
    return 0;
}