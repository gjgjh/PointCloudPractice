# NN-Trees

Python版KD-Tree和Octree，包含树的建立以及KNN搜索、Radius搜索。代码主要来自 https://github.com/lijx10/NN-Trees

## 依赖库

依赖以下第三方库，通过pip等包管理器可以安装：

- numpy
- scipy

## 示例结果

`benchmark.py`：对Brute-force、第三方库实现(scipy.spatial.KDTree)、自定义KD-Tree和自定义Octree进行测试。以`data`文件夹中的示例点云数据(来自KITTI数据集)为例，测试结果如下：

```bash
$ python benchmark.py 
Brute-force: 4.086
scipy.spatial.KDTree: build 6.191, knn 10.511
Kdtree: build 0.040, knn 2.592, radius 1.075
Octree: build 8.201, knn 2.701, radius 71.264
```

