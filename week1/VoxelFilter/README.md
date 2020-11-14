# 点云体素滤波

## 编译与运行

依赖以下第三方库：

- Eigen。矩阵相关运算，主要用于算法实现
- PCL。主要用于**可视化**

编译方法如下，编译成功后可执行文件会在`bin`文件夹中生成：

```bash
git clone https://github.com/gjgjh/PointCloudPractice
cd PointCloudPractice/week1/VoxelFilter
mkdir build
cd build
cmake ..
make -j4
```

## 示例结果

`testVoxelFilter.cpp`实现了对点云进行体素滤波。处理前图像如下图：

<img src="https://github.com/gjgjh/PointCloudPractice/blob/main/week1/VoxelFilter/support_files/testVoxelFilter1.jpg" width = 60% height = 60% />

滤波尺度voxel size设置为0.08后，结果如下图：

<img src="https://github.com/gjgjh/PointCloudPractice/blob/main/week1/VoxelFilter/support_files/testVoxelFilter2.jpg" width = 60% height = 60% />
