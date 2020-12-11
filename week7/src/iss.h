#ifndef ISS_H
#define ISS_H

#include "common.h"
#include "pointcloud.h"
#include "../Thirdparty/libnabo/nabo/nabo.h"

/**
 * Intrinsic Shape Signatures (ISS) 3D点云特征点提取算法。算法参考：
 * Zhong, Yu. "Intrinsic shape signatures: A shape descriptor for 3d object recognition." 2009 IEEE 12th International Conference on Computer Vision Workshops, ICCV Workshops. IEEE, 2009.
 */
class ISSEstimator {
public:
    ISSEstimator(const PointCloud::Ptr &data, double searchRadius = 0.1, double nonMaxRadius = 0.1, double gamma21 = 0.8, double gamma32 = 0.8, int minNeighbors = 5, double minEigVal=0.0001);

    ~ISSEstimator();

    /**
     * 开始提取ISS特征点
     * @return ISS特征点云
     */
    PointCloud::Ptr compute();

private:
    /**
     * 根据邻居点，计算一个点的协方差阵
     * @param current_index 该点的index
     * @param cov 协方差阵
     */
    void getScatterMatrix(int current_index, Eigen::Matrix3f &cov);

    double searchRadius;                    //!< 用于计算Scatter Matrix的搜索半径
    double nonMaxRadius;                    //!< 非极大值抑制半径
    double gamma_21;                        //!< 第二个特征值与第一个特征值之比的上限
    double gamma_32;                        //!< 第三个特征值与第二个特征值之比的上限
    double minEigVal;                       //!< 第三个特征值的下限
    int minNeighbors;                       //!< 非最大值抑制算法必须找到的最小邻居数

    Eigen::MatrixXf data;                   //!< 点云数据（大小 3 * numPoints）
    int numPoints;                          //!< 数据点数

    Nabo::NNSearchF *nnSearch;              //!< KNN search
};

#endif // ISS_H