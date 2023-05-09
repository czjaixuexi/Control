# 横向LQR Carsim与Simulink建模步骤

### 一.carsim选择数据库为CarSim2019.1_Data 进入后配置



### 二.配置carsim对的simulink输入输出

#### 1.点击此处进入配置

在advanced settings中输入opt_steer_ext(1) 4 车辆可以由车轮转角控制方向

![image-20221102203901891](https://gitee.com/czjaixuexi/typora_pictures/raw/master/img/image-20221102203901891.png)

#### 2.选择模型（该模型路线需要在CarSim2019.1_Data中）

![image-20230505001805998](https://gitee.com/czjaixuexi/typora_pictures/raw/master/img/image-20230505001805998.png)

#### 3.配置输入输出

![image-20221104095011138](https://gitee.com/czjaixuexi/typora_pictures/raw/master/img/image-20221104095011138.png)

##### 输入：

![image-20221102204040341](https://gitee.com/czjaixuexi/typora_pictures/raw/master/img/image-20221102204040341.png)

##### 输出：

![image-20221103153028519](https://gitee.com/czjaixuexi/typora_pictures/raw/master/img/image-20221103153028519.png)

#### 4.回到主界面，点击 send to simulink，进入Simulink界面

![image-20221102204337906](https://gitee.com/czjaixuexi/typora_pictures/raw/master/img/image-20221102204337906.png)

### 三.编辑Simulink模型

**tips:D:\CarSim2019.1\Programs\solvers\Matlab84+ （安装路径下） 有一个SolverSF可以从这里复制S-Function模块**

**在carsim中点击send to simulink后，目录中会自动生产一个simfile.sim文件(可以自己改名)，S-funciton填写该文件名。**

![image-20221102205200877](https://gitee.com/czjaixuexi/typora_pictures/raw/master/img/image-20221102205200877.png)

#### 1.配置输入输出，查看carsim导入是否成功

![image-20221103135416819](https://gitee.com/czjaixuexi/typora_pictures/raw/master/img/image-20221103135416819.png)

有了输出的结果，说明导入成功，接下来进行开始lqr模块的建模

#### 2.设置道路

1.在carsim中设置道路

![image-20221103140457321](https://gitee.com/czjaixuexi/typora_pictures/raw/master/img/image-20221103140457321.png)

![image-20221103140508441](https://gitee.com/czjaixuexi/typora_pictures/raw/master/img/image-20221103140508441.png)

![image-20221103140514457](https://gitee.com/czjaixuexi/typora_pictures/raw/master/img/image-20221103140514457.png)

2.在matlab中，先跑一遍routing_planning.m生成路径

```matlab
%绘制一个规划的路径
count=50;
[x1,y1,theta1,kr1]=straight([0,0],[20,0],0,count);
[x2,y2,theta2,kr2]=arc([20,0],[30,10],0,pi/2,count);
[x3,y3,theta3,kr3]=arc([30,10],[40,20],pi/2,0,count);
[x4,y4,theta4,kr4]=arc([40,20],[40,40],0,pi,count);
[x5,y5,theta5,kr5]=arc([40,40],[35,35],pi,3*pi/2,count);
[x6,y6,theta6,kr6]=arc([35,35],[25,35],3*pi/2,pi/2,count);
[x7,y7,theta7,kr7]=arc([25,35],[15,35],pi/2,3*pi/2,count);
[x8,y8,theta8,kr8]=arc([15,35],[5,35],3*pi/2,pi/2,count);
[x9,y9,theta9,kr9]=arc([5,35],[-15,35],pi/2,3*pi/2,count);
[x10,y10,theta10,kr10]=straight([-15,35],[-15,15],3*pi/2,count);
[x11,y11,theta11,kr11]=arc([-15,15],[0,0],3*pi/2,2*pi,count);
xr=[x1,x2,x3,x4,x5,x6,x7,x8,x9,x10,x11];
yr=[y1,y2,y3,y4,y5,y6,y7,y8,y9,y10,y11];
thetar=[theta1,theta2,theta3,theta4,theta5,theta6,theta7,theta8,theta9,theta10,theta11];
kappar=[kr1,kr2,kr3,kr4,kr5,kr6,kr7,kr8,kr9,kr10,kr11];


scatter(xr,yr);




    


function[xr,yr,thetar,kr]=straight(init_coord,end_coord,init_angle,count)
delta_x=(end_coord(1)-init_coord(1))/(count-1);
delta_y=(end_coord(2)-init_coord(2))/(count-1);
for i=1:count
    xr(i)=init_coord(1)+delta_x*i;
    yr(i)=init_coord(2)+delta_y*i;
    thetar(i)=init_angle;
    kr(i)=0;
end      
end

function[xr,yr,thetar,kr]=arc(init_coord,end_coord,init_angle,end_angle,count)
    L=sqrt((init_coord(1)-end_coord(1))^2+(init_coord(2)-end_coord(2))^2);
    R=L/sqrt(2*(1-cos(end_angle-init_angle)));
    delta_angle=(end_angle-init_angle)/(count-1) ;
  
       for i=1:count
           if delta_angle>0
               xr(i)=init_coord(1)-R*sin(init_angle)+R*sin(init_angle+delta_angle*(i-1));
               yr(i)=init_coord(2)+R*cos(init_angle)-R*cos(init_angle+delta_angle*(i-1));
               thetar(i)=init_angle+delta_angle*i;
               kr(i)=1/R;
           else
               xr(i)=init_coord(1)+R*sin(init_angle)-R*sin(init_angle+delta_angle*(i-1));

               yr(i)=init_coord(2)-R*cos(init_angle)+R*cos(init_angle+delta_angle*(i-1));
               thetar(i)=init_angle+delta_angle*i;
               kr(i)=-1/R;
           end               
       end  
end

```

![image-20221103141057593](https://gitee.com/czjaixuexi/typora_pictures/raw/master/img/image-20221103141057593.png)

#### 3.模块编写

控制模型图:

![image-20221103141207259](https://gitee.com/czjaixuexi/typora_pictures/raw/master/img/image-20221103141207259.png)

##### 1.预测模块的编写:

公式：

![image-20221103141349437](https://gitee.com/czjaixuexi/typora_pictures/raw/master/img/image-20221103141349437.png)

predict_module:

![image-20221103141644314](https://gitee.com/czjaixuexi/typora_pictures/raw/master/img/image-20221103141644314.png)

```matlab
%预测模块
function [pre_x,pre_y,pre_phi,pre_vx,pre_vy,pre_phi_dot] = fcn(x,y,phi,vx,vy,phi_dot,ts)
    pre_x=x+vx*ts*cos(phi)-vy*ts*sin(phi);
    pre_y=y+vy*ts*cos(phi)+vx*ts*sin(phi);
    pre_phi=phi+phi_dot*ts;
    pre_vx=vx;
    pre_vy=vy;
    pre_phi_dot=phi_dot;
end
```

##### 2.误差，曲率计算模块：

公式：

![image-20221103142604682](https://gitee.com/czjaixuexi/typora_pictures/raw/master/img/image-20221103142604682.png)

err_kappa_calculate_module：

**xr,yr,thetar,kappar由上一步的路径规划文件得到，使用from模块从工作区中导入**

![image-20221103142842192](https://gitee.com/czjaixuexi/typora_pictures/raw/master/img/image-20221103142842192.png)

```matlab
%误差，曲率计算模块
%xr,yr,thetar,kappar表示规划的路径
%x,y,phi,vx,vy,phi_dot为预测模块提供的车辆当前状态
function [kr,err] = fcn(x,y,phi,vx,vy,phi_dot,xr,yr,thetar,kappar)
    %先遍历寻找距离最近的点作为匹配点
    n=length(xr);
    d_min=(x-xr(1))^2+(y-yr(1))^2;
    min=1;
    for i=1:n
        d=(x-xr(i))^2+(y-yr(i))^2;
        if d<d_min
            d_min=d;
            min=i;
        end
    end
    dmin=min;
    tor=[cos(thetar(dmin));sin(thetar(dmin))];
    nor=[-sin(thetar(dmin));cos(thetar(dmin))];
    d_err=[x-xr(dmin);y-yr(dmin)];
    ed=nor'*d_err;
    es=tor'*d_err;
    %projection_point_thetar=thetar(dmin);%apollo
    projection_point_thetar=thetar(dmin)+kappar(dmin)*es;
    ed_dot=vy*cos(phi-projection_point_thetar)+vx*sin(phi-projection_point_thetar);
    %消除角度的多值性(phi+2π与phi相同)
    ephi=sin(phi-projection_point_thetar);
    %%%%%%%%%
    s_dot=vx*cos(phi-projection_point_thetar)-vy*sin(phi-projection_point_thetar);
    s_dot=s_dot/(1-kappar(dmin)*ed);
    ephi_dot=phi_dot-kappar(dmin)*s_dot;
    kr=kappar(dmin);
    err=[ed;ed_dot;ephi;ephi_dot];

end
```

##### 3.A、B计算模块+LQR_offline模块

公式：

![image-20221103144131399](https://gitee.com/czjaixuexi/typora_pictures/raw/master/img/image-20221103144131399.png)

lqr_offline moudle:

![image-20221103144350839](https://gitee.com/czjaixuexi/typora_pictures/raw/master/img/image-20221103144350839.png)

**1.先使用lqr_offline.m脚本文件，离线计算出A、B和各个vx下对应的K，导入lqr_offline模块**

```matlab
cf=-110000;
cr=cf;
m=1412;
Iz=1536.7;
a=1.015;
b=2.910-1.015;
k=zeros(5000,4);
for i=1:5000
    vx=0.01*i;
    A=[0,1,0,0;
        0,(cf+cr)/(m*vx),-(cf+cr)/m,(a*cf-b*cr)/(m*vx);
        0,0,0,1;
        0,(a*cf-b*cr)/(Iz*vx),-(a*cf-b*cr)/Iz,(a*a*cf+b*b*cr)/(Iz*vx)];
    B=[0;
        -cf/m;
        0;
        -a*cf/Iz];
    Q=1*eye(4); %Q越大，状态收敛很快，性能较好，但是输入可能很大，舒适性不好
    R=10;		%R越大，输入比较小，舒适性好，但是状态收敛比较慢
    k(i,:)=lqr(A,B,Q,R);
end
k1=k(:,1)';
k2=k(:,2)';
k3=k(:,3)';
k4=k(:,4)';
```

**2.然后在lqr_offline模块中，根据当前的vx查表，获取对应的K**

```matlab
function k  = fcn(k1,k2,k3,k4,vx)
    if abs(vx)<0.01
        k=[0,0,0,0];
    else
        index=round(vx/0.01);
        k=[k1(index),k2(index),k3(index),k4(index)];
    end
end
```



##### 4.前馈控制模块

公式：

![image-20221103145409592](https://gitee.com/czjaixuexi/typora_pictures/raw/master/img/image-20221103145409592.png)

forword_control：

![image-20221103145428159](https://gitee.com/czjaixuexi/typora_pictures/raw/master/img/image-20221103145428159.png)

```matlab
%直接套公式
function forword_angle = fcn(vx,a,b,m,cf,cr,k,kr)
    forword_angle=kr*(a+b-b*k(3)-(m*vx*vx/(a+b))*((b/cf)+(a/cr)*k(3)-(a/cr)));
end
```

##### 5.最终控制模块

公式：

![image-20221103145532541](https://gitee.com/czjaixuexi/typora_pictures/raw/master/img/image-20221103145532541.png)

last_angle：

![image-20221103145555787](https://gitee.com/czjaixuexi/typora_pictures/raw/master/img/image-20221103145555787.png)

```matlab
%直接套公式
function angle = fcn(k,err,forword_angle)

    angle=-k*err+forword_angle;
end
```

#### 最终效果：

##### **流程图：**

![image-20221103141207259](https://gitee.com/czjaixuexi/typora_pictures/raw/master/img/image-20221103141207259.png)

##### **模型：**

###### 整体：

![image-20221103150156326](https://gitee.com/czjaixuexi/typora_pictures/raw/master/img/image-20221103150156326.png)

###### 子模型：

![image-20221103145734320](https://gitee.com/czjaixuexi/typora_pictures/raw/master/img/image-20221103145734320.png)

<video src="VS Visualizer - CarSim - Baseline __ Quick Start Guide Example_ 2022-11-03 22-29-50.mp4"></video>

