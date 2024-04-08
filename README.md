# depth_camera_extrinsic_calib

​	这个仓库搭建了一个简单的深度相机外参标定工具，阅读代码之前可以先了解一下[传统相机标定原理和相机激光雷达联合标定的原理](https://github.com/hanlin-cheng/slam-study-note/blob/master/slam_theory/%E6%BF%80%E5%85%89%E9%9B%B7%E8%BE%BE%E4%B8%8E%E7%9B%B8%E6%9C%BA%E5%A4%96%E5%8F%82%E8%81%94%E5%90%88%E6%A0%87%E5%AE%9A%E8%B0%83%E7%A0%94.md)。大部分相机标定需要借助RGB相机提取彩色图像特征点。本仓库主要考虑再深度相机不包含RGB摄像头的情况下，将深度点投影成3D点云进行三维匹配标定并可视化。

## Prerequisites

### OpenCV

[http://opencv.org](http://opencv.org/)  **Tested with 4.5.0**

### Eigen3

 [http://eigen.tuxfamily.org](http://eigen.tuxfamily.org/)  **Required at least 3.1.0**.

### PCL

https://github.com/PointCloudLibrary/pcl **Tested with 1.8.1**

## Building

```cmake
git clone git@github.com:hanlin-cheng/depth_camera_extrinsic_calib.git
cd depth_camera_extrinsic_cali
mkdir build
cd build
cmake ..
make
```

## Running

```
./depth_camera_extrinsic_calib depth1 depth2 depth filter_dis iterations
```

depth1 depth2: depth image

depth: pixel depth value

filter_dis: filter point distance

iterations: icp iterations num

代码利用pcl进行了可视化，你可以通过敲击空格按键来进行下一次算法迭代并观察收敛效果。

> 雪净胡天牧马还，月明羌笛戍楼间。借问梅花何处落，风吹一夜满关山。