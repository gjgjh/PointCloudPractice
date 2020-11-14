# 聚类算法（K-Means/GMM/SpectralCluster）

## 编译与运行

依赖以下第三方库：

- Eigen。矩阵相关运算，主要用于算法实现
- pybind11。一个轻量级的仅含头文件的库，可在Python中对C++代码进行封装和调用，反之亦然
- libnabo (Thirdparty文件夹下)。用于KNN搜索

编译方法如下，编译成功后可执行文件会在`bin`文件夹中生成，库文件会在`lib`文件夹中生成：

> 注意：考虑到运行效率，算法使用C++实现。为了方便使用，同时也提供了Python版接口。

```bash
git clone https://github.com/gjgjh/PointCloudPractice
cd PointCloudPractice/week3
chmod +x build.sh
./build.sh
```

## 示例结果

- `testKmeans.py`：利用K-Means对简单的数据进行聚类，结果如下所示：

```bash
$ python testKmeans.py
[0, 0, 1, 1, 0, 1]
```

- `testGmm.py`：利用GMM对模拟数据（已知均值、协方差）进行聚类，结果如下所示：

```bash
$python testGmm.py
Predict_Mu:
[array([ 0.53817026,  0.38989283]), array([ 5.4976175 ,  2.55378078]), array([ 0.98641464,  6.98840006])]
Predict_Var:
[array([[ 1.11648549, -0.14874164],[-0.14874164,  2.48622082]]), 
array([[ 1.8473292 ,  0.10389207], [ 0.10389207,  2.31433406]]), 
array([[ 5.78473295,  0.23753469], [ 0.23753469,  2.12225166]])]
```

预测结果与真实结果（见`testGmm.py`文件中）比较接近，表明GMM算法的有效性。

- `testSpectralCluster.py`：利用Spectral Cluster对简单的数据进行聚类，结果如下所示：

```bash
$python testSpectralCluster.py
[2, 2, 0, 0, 2, 0, 1, 1, 1]
```

- `compare_cluster.py`：对K-Means、GMM算法以及sklearn自带的一些算法在模拟数据上进行对比，结果如下图所示：

注：由于目前实现的Spectral Cluster算法中使用Eigen自带的特征值分解，在数据量较大时无法快速进行聚类，因此这里没有参与对比

<img src="https://github.com/gjgjh/PointCloudPractice/blob/main/week3/support_files/clustering.png" width = 100% height = 100% />