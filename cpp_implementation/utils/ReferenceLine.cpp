/**
 * @file ReferenceLine.cpp
 * @author czj
 * @brief
 * @version 0.1
 * @date 2023-05-15
 *
 * @copyright Copyright (c) 2023
 *
 */

#include "ReferenceLine.h"
 /**
  * 构造函数，求解出参考轨迹点上的曲率等信息
  */
ReferenceLine::ReferenceLine() {
    refer = std::vector<ReferPoint>(500);
    // 生成参考轨迹
    for (int i = 0; i < refer.size(); i++) {
        refer[i].x = 0.5 * i;
        refer[i].y = 2.0 * std::sin(refer[i].x / 10.0) + 3.0 * std::cos(refer[i].x / 9.0);
    }
    double x_delta = 0.0;
    double y_delta = 0.0;
    double x_delta_2 = 0.0;
    double y_delta_2 = 0.0;
    for (int i = 0; i < refer.size(); i++) {
        if (i == 0) {
            x_delta = (refer[i + 1].x - refer[i].x);
            y_delta = (refer[i + 1].y - refer[i].y);
            x_delta_2 = (refer[i + 2].x - refer[i + 1].x) - (refer[i + 1].x - refer[i].x);
            y_delta_2 = (refer[i + 2].y - refer[i + 1].y) - (refer[i + 1].y - refer[i].y);
        }
        else if (i == refer.size() - 1) {
            x_delta = (refer[i].x - refer[i - 1].x);
            y_delta = (refer[i].y - refer[i - 1].y);
            x_delta_2 = (refer[i].x - refer[i - 1].x) - (refer[i - 1].x - refer[i - 2].x);
            y_delta_2 = (refer[i].y - refer[i - 1].y) - (refer[i - 1].y - refer[i - 2].y);
        }
        else {
            x_delta = 0.5 * (refer[i + 1].x - refer[i - 1].x);
            y_delta = 0.5 * (refer[i + 1].y - refer[i - 1].y);
            x_delta_2 = (refer[i + 1].x - refer[i].x) - (refer[i].x - refer[i - 1].x);
            y_delta_2 = (refer[i + 1].y - refer[i].y) - (refer[i].y - refer[i - 1].y);
        }
        refer[i].heading = std::atan2(y_delta, x_delta);
        //  参数方程曲率计算
        refer[i].cur = std::abs(y_delta_2 * x_delta - x_delta_2 * y_delta) / std::pow((x_delta * x_delta + y_delta * y_delta), 3 / 2);
    }
}



