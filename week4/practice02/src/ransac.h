#ifndef RANSAC_H
#define RANSAC_H

#include "common.h"

class Ransac {
public:
    /**
     * 构造函数
     * @param data 数据
     * @param distThreshold 内点距离阈值
     * @param minSizeSamplesToFit 一次采样最少需要多少点
     * @param probGoodSample 可以抽到好样本的概率
     * @param maxIter 最大迭代次数
     */
    Ransac(const std::vector<Eigen::VectorXd> &data, double distThreshold, int minSizeSamplesToFit, double probGoodSample, int maxIter);

    /**
     * 开始RANSAC过程
     * @param inliersIdx 数据中属于内点的index
     * @param model 输出模型参数
     * @return 是否找到满足条件的结果
     */
    bool execute(std::vector<int> &inliersIdx, Eigen::VectorXd &model);

protected:
    /**
     * 根据数据拟合模型
     * @param data 输入数据（个数等于minSizeSamplesToFit）
     * @param model 输出模型参数
     */
    virtual void fit(const std::vector<Eigen::VectorXd> &data, Eigen::VectorXd &model) = 0;

    /**
     * 随机生成数据，用于模型拟合
     * @return 随机产生的数据（个数等于minSizeSamplesToFit）。如果返回容器empty，表明没有产生有效的随机数据。
     */
    virtual std::vector<Eigen::VectorXd> generateData() = 0;

    /**
     * 计算一个数据点到模型的距离
     * @param data 输入数据点
     * @param model 已知模型参数
     * @return 距离
     */
    virtual double calDist(const Eigen::VectorXd &data, const Eigen::VectorXd &model) = 0;

    /**
     * 根据当前模型参数，统计所有数据点中符合内点的index
     * @param model 已知模型参数
     * @param inliersIdx 内点index
     */
    void calInlierIdx(const Eigen::VectorXd &model, std::vector<int> &inliersIdx);

    /**
     * 计算最新的RANSAC迭代次数
     * @param pInliers 当前内点概率（根据已知模型计算得）
     * @return RANSAC迭代次数
     */
    int calIteration(double pInliers);


    std::vector<Eigen::VectorXd> data;      //!< 数据
    int numPoints;                          //!< 数据点数
    int numDimensions;                      //!< 数据维度

    double distThreshold;                   //!< 内点距离阈值
    int minSizeSamplesToFit;                //!< 一次采样最少需要多少点
    double probGoodSample;                  //!< 可以抽到好样本的概率
    int maxIter;                            //!< 最大迭代次数
};

#endif // RANSAC_H