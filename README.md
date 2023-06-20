# 简介

自动驾驶的控制算法实现。



**TO_DO**：

后续计划使用C++ 通过carla-ros-bridge在carla上实现。



# 目录

## [cpp_implementation](./cpp_implementation)

此部分参考[PythonRobotics](https://github.com/AtsushiSakai/PythonRobotics#pythonrobotics),使用C++ 实现了部分横向控制算法。

### 

### 依赖

推荐在Ubuntu 18.04/20.04 环境下运行

- **cmake**

  在Ubuntu中安装cmake：

  ```
  sudo apt install cmake
  ```

- **Eigen**

  在Ubuntu中安装Eigen：

  ```
  sudo apt-get install libeigen3-dev
  ```

- **python3**



### 编译

在当前目录下输入：

```shell
mkdir build
cd build
cmake ../
make
```









### 控制效果图

蓝色为参考轨迹

红色为车辆行驶轨迹



#### Pure_Pursuit

![pure_pursuit_demo](https://gitee.com/czjaixuexi/typora_pictures/raw/master/img/202305180229501.png)

#### Rear_Wheel_Feedback

![rear_demo](https://gitee.com/czjaixuexi/typora_pictures/raw/master/img/202305180229504.png)



#### Stanley

![stanley_demo](https://gitee.com/czjaixuexi/typora_pictures/raw/master/img/202305180229505.png)





## [model](./model)

此目录存放matlab代码与模型。

此部分参考B站老王，使用carsim2019.1与Matlab2020a实现自动驾驶的横向+纵向控制算法。



## [modeling_process](./modeling_process)

此目录存放建模过程笔记：

1.  [PID纵向控制建模](./modeling_process/PID纵向控制.md) 
2.  [LQR横向控制建模](./modeling_process/横向LQR.md) 
3.  [横纵向综合控制建模](./modeling_process/横纵向综合控制.md) 





## [note](./note)

此目录存放横纵向控制相关知识笔记：

1.  [运动学+动力学模型](./note/运动学+动力学模型.md) 
2.  [纵向控制](./note/纵向控制.md) 
3.  [横向控制](./note/横向控制.md)  
4.  [转向相关知识点](./note/转向相关知识点.md) 
