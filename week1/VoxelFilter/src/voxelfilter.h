#ifndef VOXELFILTER_H
#define VOXELFILTER_H

#include "pointcloud.h"

class VoxelFilter {
public:
    enum class METHOD {
        CENTROID,                       //!< 采样时voxel的坐标通过计算centroid方式得到
        RANDOM                          //!< 采样时voxel的坐标通过随机选择一个点云坐标方式得到
    };

    /**
     * 构造函数.
     * @param pointCloud 待处理点云
     * @param voxelSize 每个voxel的size（3个方向）
     * @param method 滤波方法
     */
    VoxelFilter(const PointCloud::Ptr &pointCloud, Eigen::Vector3d voxelSize, METHOD method) : pointCloud(pointCloud), voxel_size(voxelSize), method(method) {}

    /**
     * 开始进行滤波.
     * @return 滤波处理后的点云
     */
    PointCloud::Ptr filter();

private:
    typedef std::pair<int, long long> CloudidxVoxelidx;

    void calVoxelIndex(std::vector<CloudidxVoxelidx> &index_vector);
    void calCentroid(const std::vector<CloudidxVoxelidx> &index_vector, PointCloud::Ptr& output);

    PointCloud::Ptr pointCloud;          //!< 待处理点云
    Eigen::Vector3d voxel_size;          //!< 滤波分辨率，即每个voxel的大小（3个方向）
    METHOD method;                       //!< 滤波方法

    long long dx;                        //!< voxel采样维数（x方向）
    long long dy;                        //!< voxel采样维数（y方向）
    long long dz;                        //!< voxel采样维数（z方向）
};

#endif // VOXELFILTER_H