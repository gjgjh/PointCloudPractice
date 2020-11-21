#ifndef POINTCLOUDCLUSTER_H
#define POINTCLOUDCLUSTER_H

#include "common.h"

class PointCloudCluster {
public:
    /**
     * 构造函数
     * @param data 原始数据
     * @param tolerance 聚类时的距离阈值
     * @param minClusterSize 类别最小点数
     * @param maxClusterSize 类别最大点数
     */
    PointCloudCluster(const PtCloud::Ptr &data, double tolerance = 1.0, int minClusterSize = 100, int maxClusterSize = 25000) :
            data(data), tolerance(tolerance), min_cluster_size(minClusterSize), max_cluster_size(maxClusterSize) {}

    /**
     * 开始聚类
     * @return 聚类后的点云。相同的类别颜色相同
     */
    PtCloud::Ptr cluster();

private:
    PtCloud::Ptr data;          //!< 原始数据
    double tolerance;           //!< 聚类时的距离阈值
    int min_cluster_size;       //!< 类别最小点数
    int max_cluster_size;       //!< 类别最大点数
};

#endif // POINTCLOUDCLUSTER_H