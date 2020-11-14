# 点云PCA分析

## 编译与运行

依赖以下第三方库：

- Eigen。矩阵相关运算，主要用于算法实现
- PCL。主要用于**可视化**

编译方法如下，编译成功后可执行文件会在`bin`文件夹中生成：

```bash
git clone https://github.com/gjgjh/PointCloudPractice
cd PointCloudPractice/week1/PCA
mkdir build
cd build
cmake ..
make -j4
```

## 示例结果

- `testPCA.cpp`实现了对点云进行PCA分析，找到了三个主方向。

<img src="https://github.com/gjgjh/PointCloudPractice/blob/main/week1/PCA/support_files/testPCA.jpg" width = 60% height = 60% />

- `testPCA2.cpp`实现了对点云进行PCA分析，并将点云投影到前两个主方向上。如下图所示，结果是一个平面点云。

<img src="https://github.com/gjgjh/PointCloudPractice/blob/main/week1/PCA/support_files/testPCA2.jpg" width = 60% height = 60% />

- `testPCA3.cpp`实现了对点云进行表面法向估计。其中，为了可视化效果，结果中每隔10个点云显示一个法向。

<img src="https://github.com/gjgjh/PointCloudPractice/blob/main/week1/PCA/support_files/testPCA3.jpg" width = 60% height = 60% />