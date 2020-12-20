#ifndef FPFT_H
#define FPFT_H

#include "common.h"

/**
 * 4维特征，描述了两个点之间的相对位置以及法向量信息。用于计算SPFH等
 */
struct Quadruplet {
    double f1;      //!< 特征1：theta
    double f2;      //!< 特征2：alpha
    double f3;      //!< 特征3：phi
    double f4;      //!< 特征4：d
};

/**
 * Fast Point Feature Histogram (FPFH) 特征提取
 */
class FPFHEstimation {
public:
    using Ptr = std::shared_ptr<FPFHEstimation>;

    FPFHEstimation() = default;

    /**
     * 工厂函数
     * @param numBins1 theta直方图划分的格子数
     * @param numBins2 alpha直方图划分的格子数
     * @param numBins3 phi直方图划分的格子数
     * @param radius KNN搜索半径
     * @return FPFHEstimation对象指针
     */
    static Ptr create(int numBins1 = 11, int numBins2 = 11, int numBins3 = 11, double radius = 0.1);

    /**
     * 给定某个点，计算其FPFH特征 (Fast Point Feature Histogram)
     * @param cloud 点云数据
     * @param normals 法向量数据
     * @param idx 点的index
     * @return FPFH特征，大小等于numBins1+numBins2+numBins3
     */
    std::vector<double> computeFPFH(const pcl::PointCloud<PointT>::Ptr &cloud, const pcl::PointCloud<NormalT>::Ptr &normals, int idx);

private:
    /**
     * 给定某个点，计算其SPFH特征 (Simple Point Feature Histogram)
     * @param cloud 点云数据
     * @param normals 法向量数据
     * @param idx 点的index
     * @return SPFH特征，大小等于numBins1+numBins2+numBins3
     */
    std::vector<double> computeSPFH(const pcl::PointCloud<PointT>::Ptr &cloud, const pcl::PointCloud<NormalT>::Ptr &normals, int idx);

    /**
     * 给定两个点，计算pair feature
     * @param cloud 点云数据
     * @param normals  法向量数据
     * @param p_idx 点1的index
     * @param q_idx 点2的index
     * @param quadruplet pair feature，用一个四元组表示
     */
    void computeQuadruplet(const pcl::PointCloud<PointT>::Ptr &cloud, const pcl::PointCloud<NormalT>::Ptr &normals, int p_idx, int q_idx, Quadruplet &quadruplet);

    int numBins1;                       //!< theta直方图划分的格子数
    int numBins2;                       //!< alpha直方图划分的格子数
    int numBins3;                       //!< phi直方图划分的格子数

    double searchRadius;                //!< KNN搜索半径
    pcl::KdTreeFLANN<PointT> kdtree;    //!< KdTree

    std::vector<double> descriptor;     //!< 描述子，大小为numBins1+numBins2+numBins3
};

#endif // FPFT_H