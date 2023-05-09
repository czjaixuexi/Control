# 纵向控制

## 一、更改carsim相关设置

##### 1.设置新的输入输出

##### 输入：

![image-20230509212215758](https://gitee.com/czjaixuexi/typora_pictures/raw/master/img/202305092141284.png)

##### 输出：

![image-20230509212224329](https://gitee.com/czjaixuexi/typora_pictures/raw/master/img/202305092141286.png)



##### 2.设置换挡策略，把油车模拟成电车

![image-20230509212231279](https://gitee.com/czjaixuexi/typora_pictures/raw/master/img/202305092141287.png)

![image-20230509212237240](https://gitee.com/czjaixuexi/typora_pictures/raw/master/img/202305092141288.png)

##### 3.更改道路为直线

![image-20230509212243879](https://gitee.com/czjaixuexi/typora_pictures/raw/master/img/202305092141289.png)

##### 4.send to simulink 搭建一个简单的测试模型测试输入输出

![image-20230509212251062](https://gitee.com/czjaixuexi/typora_pictures/raw/master/img/202305092141290.png)

电机的策略：

![image-20230509212258636](https://gitee.com/czjaixuexi/typora_pictures/raw/master/img/202305092141291.png)

![image-20230509212307323](https://gitee.com/czjaixuexi/typora_pictures/raw/master/img/202305092141292.png)

```matlab
function torque = fcn(thr,rpm)
Tmax=380*thr;
    if (rpm<=4523)
        torque=Tmax;
    else
        torque=Tmax*4523/rpm;
    end
end
```

## 二、油门/刹车标定表的制作

#### 注：标定后的数据已经保存在biaoding.mat中，后续可以直接加载该数据，无需再重复标定。

![image-20230509212322473](https://gitee.com/czjaixuexi/typora_pictures/raw/master/img/202305092141293.png)



通过实验，得到大量的(v,a,thr)的三维点，从而拟合出thr = f(v,a)

在apollo中通过深度学习完成标定。

#### 油门标定

##### 1.调整模型输入为thr，保存模型calibration.m,通过to workspace添加输入输出vx,ax



![image-20230509212333791](https://gitee.com/czjaixuexi/typora_pictures/raw/master/img/202305092141294.png)

##### 2.在matlab中编写脚本

先用简单的脚本测试一下

```matlab
thr =1;
sim('calibration');
```

###### 运行后发现数据成功加载到工作区

![image-20230509212341709](https://gitee.com/czjaixuexi/typora_pictures/raw/master/img/202305092141295.png)

###### 编写油门标定脚本thr_calibration.m：

```matlab
thr=0;%初始化油门
for i=1:21
    %该程序非常耗时，如果需要更多更密集的数据，请先测试
    sim('calibration');
    v_temp(:,i)=vx.data;
    a_temp(:,i)=ax.data;
    thr_temp(:,i)=ones(length(vx.data),1)*thr;
    thr=thr+0.01;
end

%合并,一定要转成行向量再合并，否则会导致合并失败
v=v_temp(:,1)';
a=a_temp(:,1)';
tr=thr_temp(:,1)';
for i=2:length(a_temp(1,:))
    v=[v,v_temp(:,i)'];
    a=[a,a_temp(:,i)'];
    tr=[tr,thr_temp(:,i)'];
end
%拟合
F=scatteredInterpolant(v',a',tr');%转成列向量
vu=0:0.1:50;
au=0:0.1:5;
table=zeros(length(vu),length(au));
for i=1:length(vu)
    for j=1:length(au)
        table(i,j)=F(vu(i),au(j));
    end
end
```

###### 运行后看到工作区已经加载了数据

![image-20230509212347724](https://gitee.com/czjaixuexi/typora_pictures/raw/master/img/202305092141296.png)

##### 3.回到simulink，添加2d lookup模块用于查找工作区的数据，并将工作区的数据填入：

![image-20230509212352668](https://gitee.com/czjaixuexi/typora_pictures/raw/master/img/202305092141297.png)

![image-20230509212358688](https://gitee.com/czjaixuexi/typora_pictures/raw/master/img/202305092141298.png)

#### 刹车标定

##### 1.在Simulink模型中将油门设置为0，刹车添加一个brake变量

![image-20230509212404496](https://gitee.com/czjaixuexi/typora_pictures/raw/master/img/202305092141299.png)

##### 2.在carsim中将车辆的速度设置为180km/h，然后send to simulink

![image-20230509212409475](https://gitee.com/czjaixuexi/typora_pictures/raw/master/img/202305092141300.png)

##### 3.编写标定脚本brake_calibration

```matlab
brake=0.1;%初始化刹车
for i=1:80
    %该程序非常耗时，如果需要更多更密集的数据，请先测试
    sim('calibration');
    v_temp(:,i)=vx.data;
    a_temp(:,i)=ax.data;
    brake_temp(:,i)=ones(length(vx.data),1)*brake;
    brake=brake+0.1;
    
end


%合并,一定要转成行向量再合并，否则会导致合并失败
vbr=v_temp(:,1)';
abr=a_temp(:,1)';
br=brake_temp(:,1)';
for i=2:length(a_temp(1,:))
    vbr=[vbr,v_temp(:,i)'];
    abr=[abr,a_temp(:,i)'];
    br=[br,brake_temp(:,i)'];
end
% 
%拟合
F=scatteredInterpolant(vbr',abr',br');%转成列向量
vubr=0:0.05:50;
aubr=-8:0.05:0;
tablebr=zeros(length(vubr),length(aubr));
for i=1:length(vubr)
    for j=1:length(aubr)
        tablebr(i,j)=F(vubr(i),aubr(j));
    end
end   
```

##### 4.回到simulink，添加2d lookup模块用于查找工作区的数据，并将工作区的数据填入：

![image-20230509212414378](https://gitee.com/czjaixuexi/typora_pictures/raw/master/img/202305092141301.png)

##### 5.将表接入模型，做一下简单测试

![image-20230509212418948](https://gitee.com/czjaixuexi/typora_pictures/raw/master/img/202305092141302.png)

vx图像:

![image-20230509212425224](https://gitee.com/czjaixuexi/typora_pictures/raw/master/img/202305092141303.png)



## 三、双PID控制

##### 标定，将油门和刹车标定表合成一张表。 

**注：标定后的数据已经保存在biaoding.mat中，后续可以直接加载该数据，无需再重复标定。**

标定模型：

![image-20230509212430939](https://gitee.com/czjaixuexi/typora_pictures/raw/master/img/202305092141304.png)

其中biaoding moudle根据x的正负，判断输入是油门还是刹车：

```matlab
function [thr,brake] = fcn(x)
    %正数代表油门，负数代表刹车
    %不允许同时踩油门和刹车
    if x>=0
        thr=x;
        brake=0;
    else
        thr=0;
        brake=-x;
    end
```

###### 油门标定脚本thr_calibration.m:

启动前查看carsim车辆车速是否为0

```matlab
%%%启动前检查carsim的初速度是否为0
x=0;%初始化油门
for i=1:21
    %该程序非常耗时，如果需要更多更密集的数据，请先测试
    sim('calibration');
    v_temp(:,i)=vx.data;
    a_temp(:,i)=ax.data;
    thr_temp(:,i)=ones(length(vx.data),1)*x;        
    x=x+0.05;
end

%合并,一定要转成行向量再合并，否则会导致合并失败
 v=v_temp(:,1)';
 a=a_temp(:,1)';
 tr=thr_temp(:,1)';
for i=2:length(v_temp(1,:))
    v=[v,v_temp(:,i)'];
    a=[a,a_temp(:,i)'];
    tr=[tr,thr_temp(:,i)'];
end
```

###### 刹车标定脚本brake_calibration.m:  

启动前查看carsim车辆车速是否为180

```matlab
%启动前检查车的初速度是否为180
x=0;%初始化刹车
%%刹车的初速度一定要比较高，180km/h、144km/h
for i=1:81
    %该程序非常耗时，如果需要更多更密集的数据，请先测试
    sim('calibration');
    v_temp1(:,i)=vx.data;
    a_temp1(:,i)=ax.data;
    brake_temp1(:,i)=ones(length(vx.data),1)*x;
    %%%%这里是消除奇异性，因为无论brake=1还是2，最后都会导致车的v，a=0；这将导致多值性
    for j=1:length(v_temp1(:,i))
        if v_temp1(j,i)<0.01
            brake_temp1(j,i)=0;
        end        
    end
  
    x=x-0.1;   
end
a_temp1(1,:)=a_temp1(2,:);

%合并,一定要转成行向量再合并，否则会导致合并失败
vbr=v_temp1(:,1)';
abr=a_temp1(:,1)';
br=brake_temp1(:,1)';
for i=2:length(v_temp1(1,:))
    vbr=[vbr,v_temp1(:,i)'];
    abr=[abr,a_temp1(:,i)'];
    br=[br,brake_temp1(:,i)'];
end
```

###### 将油门、刹车标定表合成为一个表的脚本generate_calibration.m:

```matlab
v2=[v,vbr];
a2=[a,abr];
br2=[tr,br];


F=scatteredInterpolant(v2',a2',br2');%转成列向量
vubr=0:0.05:50;
aubr=-8:0.05:5;
tablebr=zeros(length(vubr),length(aubr));
for i=1:length(vubr)
    for j=1:length(aubr)
        tablebr(i,j)=F(vubr(i),aubr(j));
    end
end
```



###### 在simulink中添加2D look up：

![image-20230509212437640](https://gitee.com/czjaixuexi/typora_pictures/raw/master/img/202305092141305.png)



![image-20230509212443368](https://gitee.com/czjaixuexi/typora_pictures/raw/master/img/202305092141306.png)

##### Simulink模型：

###### 将下面3个模块合为一个subsystem

![image-20230509212449399](https://gitee.com/czjaixuexi/typora_pictures/raw/master/img/202305092141307.png)

###### 合成后：

![image-20230509212453512](https://gitee.com/czjaixuexi/typora_pictures/raw/master/img/202305092141308.png)

###### 输入期望速度，不加PID观察效果：

![image-20230509212501782](https://gitee.com/czjaixuexi/typora_pictures/raw/master/img/202305092141309.png)

速度存在稳态误差：

![image-20230509212510493](https://gitee.com/czjaixuexi/typora_pictures/raw/master/img/202305092141310.png)

###### 添加pid模块：

![image-20230509212516352](https://gitee.com/czjaixuexi/typora_pictures/raw/master/img/202305092141311.png)

速度控制得到优化：

![image-20230509212522003](https://gitee.com/czjaixuexi/typora_pictures/raw/master/img/202305092141312.png)

##### 双PID控制：

###### 首先添加一个随时间变化的规划模块，用来提供期望的位置、速度、加速度：

![image-20230509212526455](https://gitee.com/czjaixuexi/typora_pictures/raw/master/img/202305092141313.png)

planning:

提供提供期望的位置、速度、加速度

```matlab
function [s,v,a] = fcn(t)
if t<10
    s=0.1*t^3/3;
    v=0.1*t^2;
    a=0.2*t;
else
    a=2-0.1*(t-10);
    v=2*t-0.05*(t-10)^2-10;
    s=t^2-0.05*(t-10)^3/3-10*t+100/3;
end
```

###### 在carsim的输出中添加车的纵向位置信息：

![image-20230509212530738](https://gitee.com/czjaixuexi/typora_pictures/raw/master/img/202305092141314.png)

###### 添加双PID控制：

原理：通过控制油门/刹车来控制车辆的加速度，实现加速度、速度、位置的控制。

控制流程图： 先对位置进行PID，给出速度补偿，再对速度进行PID，给出加速度补偿。

加速度=期望加速度+加速度补偿。

![image-20230509212544821](https://gitee.com/czjaixuexi/typora_pictures/raw/master/img/202305092141315.png)





###### 打包：

![image-20230509212552835](https://gitee.com/czjaixuexi/typora_pictures/raw/master/img/202305092141316.png)

运行模型后发现，加速器、速度、位置曲线与期望值拟合较好，纵向双PID控制完成。

![image-20230509212559811](https://gitee.com/czjaixuexi/typora_pictures/raw/master/img/202305092141317.png)