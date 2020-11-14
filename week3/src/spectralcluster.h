#ifndef SPECTRALCLUSTER_H
#define SPECTRALCLUSTER_H

#include "common.h"
#include "kmeans.h"
#include "../Thirdparty/libnabo/nabo/nabo.h"

class SpectralCluster {
public:
    /**
     * 构造函数
     * @param k 聚类个数
     * @param numNeighbors 建立Adjacency Matrix时使用的KNN邻居个数
     * @param tolerance 中心点最大误差。用于K-means过程
     * @param maxIter 最大迭代次数
     */
    SpectralCluster(int k = 2, int numNeighbors = 2, double tolerance = 0.0001, int maxIter = 300);

    /**
     * 析构函数
     */
    virtual ~SpectralCluster();

    /**
     * 开始进行聚类
     * @param data 输入数据。需要保证数据非空，且所有数据维度保持一致
     */
    void fit(const std::vector<Eigen::VectorXf> &dataset);

    /**
     * 根据聚类结果，预测输入数据的类别
     * 注意，由于spectral cluster法不能直接预测新的数据（只能预测训练时的原数据），因此这里采用nearest prediction策略
     * @param data 输入数据
     * @return 每个输入数据对应的预测类别
     */
    std::vector<int> predict(const std::vector<Eigen::VectorXf> &dataset);

    /**
     * 对输入数据进行训练并预测（即预测时使用原数据集）
     * @param data 输入数据
     * @return 每个输入数据对应的预测类别
     */
    std::vector<int> fit_predict(const std::vector<Eigen::VectorXf> &dataset);

private:
    /**
     * 根据KNN算法，计算Adjacency Matrix
     */
    void calAdjacencyMatrix();

    /**
     * 根据Adjacency Matrix计算Degree Matrix
     */
    void calDegreeMatrix();

    /**
     * 根据Adjacency Matrix和Degree Matrix计算Laplacian Matrix
     */
    void calLaplacianMatrix();

    /**
     * 对Laplacian Matrix进行特征分解，并返回前k个最小的特征向量
     * @param matrix Laplacian Matrix
     * @return 前k个最小的特征向量
     */
    std::vector<Eigen::VectorXd> doDecomposition(const Eigen::MatrixXf &matrix);

    void sortEigenVectorByValues(Eigen::VectorXf &eigenValues, Eigen::MatrixXf &eigenVectors);


    int k;                                  //!< 聚类个数
    int numNeighbors;                       //!< 建立Adjacency Matrix时使用的KNN邻居个数
    double tolerance;                       //!< 中心点最大误差
    int maxIter;                            //!< 最大迭代次数

    Eigen::MatrixXf data;                   //!< 数据（大小numDimensions * numPoints）
    int numPoints;                          //!< 数据点数
    int numDimensions;                      //!< 数据维度
    std::vector<int> clusters;              //!< 预测类别

    Eigen::MatrixXf adjacencyMatrix;        //!< Adjacency Matrix，用于描述样本之间的相似程度
    Eigen::MatrixXf degreeMatrix;           //!< Degree Matrix，用于辅助计算
    Eigen::MatrixXf laplacianMatrix;        //!< Laplacian Matrix，用于辅助计算

    Nabo::NNSearchF *nnSearch;              //!< KNN search
    Kmeans *kmeans;                         //!< 内部实现需要依赖的Kmeans类
};

#endif // SPECTRALCLUSTER_H