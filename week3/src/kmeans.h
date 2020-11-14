#ifndef KMEANS_H
#define KMEANS_H

#include "common.h"

class Kmeans {
public:
    /**
     * 构造函数
     * @param k 聚类个数
     * @param tolerance 中心点最大误差
     * @param maxIter 最大迭代次数
     */
    Kmeans(int k = 2, double tolerance = 0.0001, int maxIter = 300) : k(k), tolerance(tolerance), maxIter(maxIter) {}

    /**
     * 开始进行聚类
     * @param data 输入数据。需要保证数据非空，且所有数据维度保持一致
     */
    void fit(const std::vector<Eigen::VectorXd> &data);

    /**
     * 根据聚类结果，预测输入数据的类别
     * @param data 输入数据
     * @return 每个输入数据对应的预测类别
     */
    std::vector<int> predict(const std::vector<Eigen::VectorXd> &data);

private:
    /**
     * 初始聚类中心
     */
    void initialCentroids();

    /**
     * 计算各类的中心
     * @return 当前中心点误差
     */
    double computeCentroids();

    /**
     * 根据聚类中心结果，计算最新的聚类情况
     */
    void computeClusters();

    int k;                                  //!< 聚类个数
    double tolerance;                       //!< 中心点最大误差
    int maxIter;                            //!< 最大迭代次数

    std::vector<Eigen::VectorXd> data;      //!< 数据
    int numDimensions;                      //!< 数据维度

    std::vector<std::vector<int>> clusters; //!< k个聚类，每个聚类的点号（data中的index）
    std::vector<Eigen::VectorXd> centroids; //!< k个聚类中心
};

#endif // KMEANS_H