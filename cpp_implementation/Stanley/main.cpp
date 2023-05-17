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

#include "Stanley.h"
#include "../utils/matplotlibcpp.h"

namespace plt = matplotlibcpp;

#define PI 3.1415926


int main() {
    double x = 0.0, y = 0.0, yaw = 0.0, v = 30.0 / 3.6, l = 2.9, dt = 0.1;
    // 车辆状态
    State state(x, y, yaw, v, l);
    // 车辆模型
    VehicleModel vehicle(state, dt);
    // 生成参考轨迹
    ReferenceLine refer_line;
    //后面画图用
    std::vector<double> refer_x(refer_line.refer.size());
    std::vector<double> refer_y(refer_line.refer.size());
    for (int i = 0;i < refer_line.refer.size();i++) {
        refer_x[i] = refer_line.refer[i].x;
        refer_y[i] = refer_line.refer[i].y;
    }

    double k = 2.0; //增益系数
    // 车辆运动轨迹
    std::vector<double> trajectory_x, trajectory_y;
    Stanley stanley;
    while (true) {
        plt::clf();
        double delta = stanley.stanley_control(vehicle.state, refer_line.refer, k);
        vehicle.updateState(0, delta);
        trajectory_x.push_back(vehicle.state.x);
        trajectory_y.push_back(vehicle.state.y);
        //画图
        plt::plot(refer_x, refer_y, "b--");
        plt::plot(trajectory_x, trajectory_y, "r");
        plt::grid(true);
        plt::pause(0.01);
        // 到达终点，退出控制
        if (stanley.calc_distance(vehicle.state, refer_line.refer, refer_line.refer.size() - 1) < 0.5) {
            std::cout << "Control completion" << std::endl;
            break;
        }
    }

    // 保存图片
    const char* filename = "./stanley_demo.png";
    std::cout << "Saving result to " << filename << std::endl;
    plt::save(filename);
    plt::show();
    return 0;
}