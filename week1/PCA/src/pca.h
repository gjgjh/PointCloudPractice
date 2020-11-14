#ifndef PCA_H
#define PCA_H

#include "common.h"

class PCA{
public:
    /**
     * 构造函数.
     * @param data 输入数据. 容器的大小为数据个数，VectorXd的维度为数据维度
     */
    PCA(const std::vector<Eigen::VectorXd>& data);

    /**
     * 投影向量到主成分空间
     * @param vec 原向量
     * @return 投影到主成分空间的向量
     */
    Eigen::VectorXd project(const Eigen::VectorXd& vec)const;

    /**
     * 从主成分变换后的向量恢复原向量
     * @param vec 主成分变换后的向量
     * @return 原向量
     */
    Eigen::VectorXd backProject(const Eigen::VectorXd& vec)const;

    Eigen::VectorXd eigenvalues;        //!< 特征值
    Eigen::MatrixXd eigenvectors;       //!< 特征向量，即各主成分向量
    Eigen::VectorXd mean;               //!< 均值向量

    int number;                         //!< 数据个数
    int dim;                            //!< 维数

private:
    Eigen::VectorXd calMean(const std::vector<Eigen::VectorXd>& data);
    Eigen::MatrixXd subtract(const std::vector<Eigen::VectorXd>& data, const Eigen::VectorXd& mean);
    void sortEigenVectorByValues(Eigen::VectorXd& eigenValues, Eigen::MatrixXd& eigenVectors);
};

#endif // PCA_H