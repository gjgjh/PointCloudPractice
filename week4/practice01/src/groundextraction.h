#ifndef GROUNDEXTRACTION_H
#define GROUNDEXTRACTION_H

#include "common.h"

class GroundExtraction {
public:
    /**
     * 构造函数
     * @param data 输入原始点云数据
     * @param minGroundPointNum 检测的地面点云最小数量
     * @param maxIter RANSAC最大迭代次数
     * @param distThreshold RANSAC内点距离阈值
     */
    GroundExtraction(const PtCloud::Ptr &data, int minGroundPointNum, int maxIter = 100, double distThreshold = 0.05) :
            data(data), minGroundPointNum(minGroundPointNum), maxIter(maxIter), distThreshold(distThreshold) {}

    /**
     * 开始地面点云提取
     */
    void startExtraction();

    const PtCloud::Ptr &getNonGround() const;

    const PtCloud::Ptr &getGround() const;

private:
    PtCloud::Ptr data;                          //!< 原始点云数据
    int minGroundPointNum;                      //!< 检测的地面点云最小数量
    int maxIter;                                //!< RANSAC最大迭代次数
    double distThreshold;                       //!< RANSAC内点距离阈值

    PtCloud::Ptr non_ground;                    //!< 非地面点云数据
    PtCloud::Ptr ground;                        //!< 地面点云数据
};

#endif // GROUNDEXTRACTION_H