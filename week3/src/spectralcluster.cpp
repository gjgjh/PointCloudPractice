#include "spectralcluster.h"


SpectralCluster::SpectralCluster(int k, int numNeighbors, double tolerance, int maxIter) :
        k(k), numNeighbors(numNeighbors), tolerance(tolerance), maxIter(maxIter) {
    // TODO: 使用特征分析自动确定聚类个数k
}

SpectralCluster::~SpectralCluster() {
    delete nnSearch;
    delete kmeans;
}

void SpectralCluster::fit(const std::vector<Eigen::VectorXf> &dataset) {
    // 检查输入有效性
    assert(dataset.size() > 0 && k <= dataset.size() && "Data not valid\n");
    numDimensions = dataset[0].size();
    numPoints = dataset.size();
    data = Eigen::MatrixXf::Zero(numDimensions, numPoints);
    for (int i = 0; i < numPoints; ++i) {
        if (dataset[i].size() != numDimensions) {
            std::cerr << "Data dimension not match\n";
            return;
        } else {
            data.col(i) = dataset[i];
        }
    }

    // 计算adjacency matrix和degree matrix
    calAdjacencyMatrix();
    calDegreeMatrix();

    // 计算laplacian matrix
    calLaplacianMatrix();

    // 特征分解
    std::vector<Eigen::VectorXd> eigenVectors = doDecomposition(laplacianMatrix);

    // 对特征向量进行K-means聚类
    kmeans = new Kmeans(k, tolerance, maxIter);
    kmeans->fit(eigenVectors);
    clusters = kmeans->predict(eigenVectors);
}

std::vector<int> SpectralCluster::predict(const std::vector<Eigen::VectorXf> &dataset) {
    assert(!clusters.empty() && "Should call fit() first\n");

    // 检查输入有效性
    for (int i = 0; i < numPoints; ++i) {
        if (dataset[i].size() != numDimensions) {
            std::cerr << "Data dimension not match\n";
            return std::vector<int>();
        }
    }

    // 预测。选择nearest prediction作为预测结果
    int size = dataset.size();
    std::vector<int> ret(size, 0);
    for (int i = 0; i < size; ++i) {
        Eigen::VectorXi indices(numNeighbors);
        Eigen::VectorXf dists2(numNeighbors);
        nnSearch->knn(dataset[i], indices, dists2, numNeighbors);

        int nearestNeighborIdx = indices[0];

        ret[i] = clusters[nearestNeighborIdx];
    }

    return ret;
}

std::vector<int> SpectralCluster::fit_predict(const std::vector<Eigen::VectorXf> &dataset) {
    fit(dataset);
    return clusters;
}

void SpectralCluster::calAdjacencyMatrix() {
    adjacencyMatrix = Eigen::MatrixXf::Zero(numPoints, numPoints);

    // 建立KD-Tree
    nnSearch = Nabo::NNSearchF::createKDTreeLinearHeap(data);

    // 建立Adjacency Matrix
    for (int i = 0; i < numPoints; ++i) {
        Eigen::VectorXi indices(numNeighbors);
        Eigen::VectorXf dists2(numNeighbors);
        nnSearch->knn(data.col(i), indices, dists2, numNeighbors);

        for (int j = 0; j < numNeighbors; ++j) {
            int neighborIdx = indices[j];
            if (i >= neighborIdx) continue;

            double weight = 1. / dists2[j];     // TODO: 使用高斯函数等进行加权
            adjacencyMatrix(i, neighborIdx) = weight;
            adjacencyMatrix(neighborIdx, i) = weight;
        }
    }
}

void SpectralCluster::calDegreeMatrix() {
    auto row_sum = adjacencyMatrix.rowwise().sum();
    degreeMatrix = row_sum.asDiagonal();
}

void SpectralCluster::calLaplacianMatrix() {
    // 归一化Laplacian Matrix
    Eigen::MatrixXf eye = Eigen::VectorXf::Ones(numPoints).asDiagonal();
    laplacianMatrix = eye - degreeMatrix.inverse() * adjacencyMatrix;
}

std::vector<Eigen::VectorXd> SpectralCluster::doDecomposition(const Eigen::MatrixXf &matrix) {
	// TODO: 使用大型矩阵特征分解
    Eigen::EigenSolver<Eigen::MatrixXf> es(laplacianMatrix);
    Eigen::VectorXf eigenvalues = es.eigenvalues().real();
    Eigen::MatrixXf eigenvectors = es.eigenvectors().real();
    sortEigenVectorByValues(eigenvalues, eigenvectors);

    // 对于每个样本点，选择前k个最小的特征向量
    Eigen::MatrixXf first_k_eigenVectors = eigenvectors.block(0, 0, numPoints, k);

    std::vector<Eigen::VectorXd> ret;
    ret.reserve(numPoints);
    for (int i = 0; i < numPoints; ++i) {
        Eigen::VectorXd temp = first_k_eigenVectors.row(i).cast<double>();
        ret.push_back(temp);
    }

    return ret;
}

void SpectralCluster::sortEigenVectorByValues(Eigen::VectorXf &eigenValues, Eigen::MatrixXf &eigenVectors) {
    std::vector<std::pair<double, Eigen::VectorXf>> eigenValueAndVector;

    int size = eigenValues.size();
    eigenValueAndVector.reserve(size);

    for (int i = 0; i < size; ++i)
        eigenValueAndVector.push_back({eigenValues[i], eigenVectors.col(i)});

    // 从小到大排序
    std::sort(eigenValueAndVector.begin(), eigenValueAndVector.end(),
              [](const std::pair<double, Eigen::VectorXf> &pair1, const std::pair<double, Eigen::VectorXf> &pair2) -> bool {
                  return pair1.first < pair2.first;
              });

    for (int i = 0; i < size; ++i) {
        eigenValues[i] = eigenValueAndVector[i].first;
        eigenVectors.col(i).swap(eigenValueAndVector[i].second);
    }
}
