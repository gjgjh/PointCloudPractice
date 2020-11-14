#ifndef GMM_H
#define GMM_H

#include "common.h"

class GMM {
public:
    /**
     * 构造函数
     * @param k 聚类个数
     * @param epsilon 迭代终止条件：前后两次log likelihood相对变化值小于epsilon
     * @param maxIter 最大迭代次数
     */
    GMM(int k = 2, double epsilon = 0.001, int maxIter = 100) : k(k), epsilon(epsilon), maxIter(maxIter) {}

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

    const std::vector<Eigen::VectorXd> &getMu() const {
        return Mu;
    }

    const std::vector<Eigen::MatrixXd> &getVar() const {
        return Var;
    }

private:
    /**
     * 初始化参数：Mu, Var, Pi, W
     */
    void initialParameters();

    /**
     * 根据Mu, Var, Pi更新W，同时计算log likelihood
     * @return log likelihood。值越小表示参数估计越符合数据分布，可用于判断迭代终止条件
     */
    double updateW();

    /**
     * 根据W更新Pi
     */
    void updatePi();

    /**
     * 根据W更新Mu
     */
    void updateMu();

    /**
     * 根据W更新Var
     */
    void updateVar();

    /**
     * 计算数据点在已知多元高斯分布下的概率（输入数据各维度需要匹配)
     * @param x 输入数据
     * @param meanVec 均值
     * @param covMat 协方差阵
     * @return 概率值
     */
    double evalMultivNorm(const Eigen::VectorXd &x, const Eigen::VectorXd &meanVec, const Eigen::MatrixXd &covMat);


    int k;                                  //!< 聚类个数
    double epsilon;                         //!< 迭代终止条件：前后两次log likelihood相对变化值小于epsilon
    int maxIter;                            //!< 最大迭代次数

    std::vector<Eigen::VectorXd> data;      //!< 数据
    int numPoints;                          //!< 数据点数
    int numDimensions;                      //!< 数据维度

    std::vector<Eigen::VectorXd> Mu;        //!< k个聚类分布的均值
    std::vector<Eigen::MatrixXd> Var;       //!< k个聚类分布的方差
    std::vector<double> Pi;                 //!< k个聚类分布的权重
    std::vector<std::vector<double>> W;     //!< 每个样本属于每一类的概率，大小为 numPoints*k
};

#endif // GMM_H