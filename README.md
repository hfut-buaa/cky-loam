# cky-LOAM

## 1、cky-LOAM概述

cky-LOAM是主要为了自学激光SLAM算法，而写出了一个激光SLAM的简单框架，主要思想来源于LOAM算法系列（LOAM,A-LOAM,LEGO-LOAM）。

cky-LOAM利用多种工具库（Eigen,PCL,ROS,Ceres,Gtsam,g2o）简化了SLAM程序，十分方便学习与试验分析。

cky-LOAM主要包括5个部分，内容如下。

（1）点云索引重建

（2）点云特征提取

（3）里程计计算（可选ceres自动求导和解析求导两个部分）

（4）里程计因子图优化(可选g2o和gtsam两个版本）

（5）里程计闭环优化





## 2、环境依赖



（1）Ubuntu 18.04

（2）ROS Melodic （里面包含ROS,TF,PCL,Eigen,rviz,rqt等）

（3）Eigen 3.3.7

（4）Gtsam 4.0.2

（5）Ceres 2.0.0

 (6) g2o   1.14.0

（7）sophus




## 3、编译运行

下载cky-LOAM工程，并将此文件夹作为ROS功能包，放入自己建立的ROS工程中，编译并运行。


打开一个终端，输入下面两条命令，启动ckyloam程序。

```
source devel/setup.bash
roslaunch ckyloam run.launch
```

打开另一个终端，输入下面命令，运行数据。

```
rosbag play your.bag
```









