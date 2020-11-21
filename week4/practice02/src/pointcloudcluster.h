#ifndef POINTCLOUDCLUSTER_H
#define POINTCLOUDCLUSTER_H

#include "common.h"
#include "pointcloud.h"
#include "../Thirdparty/libnabo/nabo/nabo.h"

class PointCloudCluster {
public:
    /**
     * 构造函数
     * @param data 原始数据
     * @param tolerance 聚类时的距离阈值
     * @param minClusterSize 类别最小点数
     * @param maxClusterSize 类别最大点数
     */
    PointCloudCluster(const PointCloud::Ptr &cloud, double tolerance = 1.0, int minClusterSize = 100, int maxClusterSize = 25000);

    /**
     * 析构函数
     */
    virtual ~PointCloudCluster();

    /**
     * 开始聚类。时间复杂度 O(nlogn)
     * 聚类算法实现部分参考：https://github.com/PointCloudLibrary/pcl/blob/master/segmentation/include/pcl/segmentation/impl/extract_clusters.hpp
     * @return 聚类后对应的label，与data中数据一一对应。-1表示没有归为类别，0~N表示类别具体编号。
     */
    std::vector<int> cluster();

private:
    /**
     * DFS遍历点云，寻找相近的点云作为一类
     * @param index 当前点的index
     * @param processed 记录data中哪些点被访问过
     * @param indices 属于当前类别的点集合
     */
    void dfs(int index, std::vector<bool> &processed, std::vector<int> &indices);

    Eigen::MatrixXf data;                    //!< 数据（大小numDimensions * numPoints）
    int numPoints;                           //!< 数据点数
    int numDimensions;                       //!< 数据维度

    double tolerance;                        //!< 聚类时的距离阈值
    int min_cluster_size;                    //!< 类别最小点数
    int max_cluster_size;                    //!< 类别最大点数

    Nabo::NNSearchF *nnSearch;               //!< KNN search
};

#endif // POINTCLOUDCLUSTER_H