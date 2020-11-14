#include "pca.h"

PCA::PCA(const std::vector<Eigen::VectorXd> &data) {
    assert(!data.empty() && "Data is empty\n");
    number = data.size();
    dim = data[0].rows();

    // 计算均值
    mean = calMean(data);

    // 计算协方差阵
    auto normalized = subtract(data, mean);
    auto covar = 1. / (number - 1) * normalized * normalized.transpose();

    // 计算主成分
    Eigen::EigenSolver<Eigen::MatrixXd> es(covar);
    eigenvalues = es.eigenvalues().real();
    eigenvectors = es.eigenvectors().real();
    sortEigenVectorByValues(eigenvalues, eigenvectors);
}

Eigen::VectorXd PCA::project(const Eigen::VectorXd &vec) const {
    assert(vec.rows() == dim && "Dimension not matching\n");
    return eigenvectors.transpose() * vec;
}

Eigen::VectorXd PCA::backProject(const Eigen::VectorXd &vec) const {
    assert(vec.rows() == dim && "Dimension not matching\n");
    return eigenvectors * vec;
}

Eigen::VectorXd PCA::calMean(const std::vector<Eigen::VectorXd> &data) {
    Eigen::VectorXd sum = Eigen::VectorXd::Zero(dim, 1);
    for (int i = 0; i < number; ++i)
        sum += data[i];

    return 1. / number * sum;
}

Eigen::MatrixXd PCA::subtract(const std::vector<Eigen::VectorXd> &data, const Eigen::VectorXd &mean) {
    Eigen::MatrixXd substracted = Eigen::MatrixXd::Zero(dim, number);
    for (int i = 0; i < number; ++i)
        substracted.col(i) = data[i] - mean;

    return substracted;
}

void PCA::sortEigenVectorByValues(Eigen::VectorXd &eigenValues, Eigen::MatrixXd &eigenVectors) {
    std::vector<std::pair<double, Eigen::VectorXd>> eigenValueAndVector;

    int size = eigenValues.size();
    eigenValueAndVector.reserve(size);

    for (int i = 0; i < size; ++i)
        eigenValueAndVector.push_back({eigenValues[i], eigenVectors.col(i)});

    // 从大到小排序
    std::sort(eigenValueAndVector.begin(), eigenValueAndVector.end(),
              [](const std::pair<double, Eigen::VectorXd> &pair1, const std::pair<double, Eigen::VectorXd> &pair2) -> bool {
                  return pair1.first > pair2.first;
              });

    for (int i = 0; i < size; ++i) {
        eigenValues[i] = eigenValueAndVector[i].first;
        eigenVectors.col(i).swap(eigenValueAndVector[i].second);
    }
}
