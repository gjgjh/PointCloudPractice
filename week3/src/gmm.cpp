#include "gmm.h"

void GMM::fit(const std::vector<Eigen::VectorXd> &data) {
    // 检查输入有效性
    assert(data.size() > 0 && k <= data.size() && "Data not valid\n");
    numDimensions = data[0].size();
    numPoints = data.size();
    for (auto &item:data) {
        if (item.size() != numDimensions) {
            std::cerr << "Data dimension not match\n";
            return;
        }
    }
    this->data = data;

    // 初始化参数
    initialParameters();

    // 迭代EM过程
    bool converge = false;
    int iter = 0;
    double lastLogLikelihood = std::numeric_limits<double>::min();
    while (!converge) {
        // E step
        double currentLogLikelihood = updateW();

        // M step
        updatePi();
        updateMu();
        updateVar();

        if (abs(currentLogLikelihood - lastLogLikelihood) < epsilon || iter == maxIter) converge = true;
        lastLogLikelihood = currentLogLikelihood;
        ++iter;
    }
}

std::vector<int> GMM::predict(const std::vector<Eigen::VectorXd> &data) {
    assert(!Mu.empty() && !Var.empty() && !Pi.empty() && "Should call fit() first\n");

    // 检查输入有效性
    if (data.empty()) return std::vector<int>();
    for (auto &item:data) {
        if (item.size() != numDimensions) {
            std::cerr << "Data dimension not match\n";
            return std::vector<int>();
        }
    }

    // 预测
    int size = data.size();
    std::vector<int> ret(size, 0);
    for (int i = 0; i < size; ++i) {
        int clusterIdx = 0;
        double maxProb = std::numeric_limits<double>::min();

        // 寻找概率最大的类别
        for (int j = 0; j < k; ++j) {
            double prob = evalMultivNorm(data[i], Mu[j], Var[j]);
            if (prob > maxProb) {
                maxProb = prob;
                clusterIdx = j;
            }
        }

        ret[i] = clusterIdx;
    }

    return ret;
}

void GMM::initialParameters() {
    Mu = std::vector<Eigen::VectorXd>(k, Eigen::VectorXd::Zero(numDimensions, 1));
    Var = std::vector<Eigen::MatrixXd>(k, Eigen::MatrixXd::Zero(numDimensions, numDimensions));
    W = std::vector<std::vector<double>>(numPoints, std::vector<double>(k, 1));
    Pi = std::vector<double>(k, 0);

    Eigen::MatrixXd diag = Eigen::MatrixXd::Zero(numDimensions, numDimensions);
    for (int i = 0; i < numDimensions; ++i) diag(i, i) = 1;

    srand(time(NULL));
    for (int i = 0; i < k; ++i) {
        int randomIdx = rand() % data.size();
        Mu[i] = data[randomIdx];
        Var[i] = diag;
    }

    for (int i = 0; i < numPoints; ++i) {
        for (int j = 0; j < k; ++j)
            W[i][j] /= k;
    }

    updatePi();
}

double GMM::updateW() {
    double logLikelihood = 0;
    for (int i = 0; i < numPoints; ++i) {
        double sum = 0;
        for (int j = 0; j < k; ++j) {
            W[i][j] = Pi[j] * evalMultivNorm(data[i], Mu[j], Var[j]);
            sum += W[i][j];
        }

        // 归一化
        for (int j = 0; j < k; ++j) W[i][j] /= sum;

        logLikelihood += log(sum);
    }

    return logLikelihood;
}

void GMM::updatePi() {
    for (int i = 0; i < numPoints; ++i) {
        for (int j = 0; j < k; ++j)
            Pi[j] += W[i][j];
    }

    // 归一化
    for (int i = 0; i < k; ++i) Pi[i] /= numPoints;
}

void GMM::updateMu() {
    Mu = std::vector<Eigen::VectorXd>(k, Eigen::VectorXd::Zero(numDimensions, 1));
    for (int i = 0; i < numPoints; ++i) {
        for (int j = 0; j < k; ++j)
            Mu[j] += W[i][j] * data[i];
    }

    // 归一化
    for (int i = 0; i < k; ++i) Mu[i] /= (numPoints * Pi[i]);
}

void GMM::updateVar() {
    Var = std::vector<Eigen::MatrixXd>(k, Eigen::MatrixXd::Zero(numDimensions, numDimensions));
    for (int i = 0; i < numPoints; ++i) {
        for (int j = 0; j < k; ++j)
            Var[j] += W[i][j] * (data[i] - Mu[j]) * (data[i] - Mu[j]).transpose();
    }

    // 归一化
    for (int i = 0; i < k; ++i) Var[i] /= (numPoints * Pi[i]);
}

// 参考：https://stackoverflow.com/questions/41538095/evaluate-multivariate-normal-gaussian-density-in-c
double GMM::evalMultivNorm(const Eigen::VectorXd &x, const Eigen::VectorXd &meanVec, const Eigen::MatrixXd &covMat) {
    const double logSqrt2Pi = 0.5 * std::log(2 * M_PI);
    typedef Eigen::LLT<Eigen::MatrixXd> Chol;
    Chol chol(covMat);

    if (chol.info() != Eigen::Success) throw "decomposition failed!";
    const Chol::Traits::MatrixL &L = chol.matrixL();
    double quadform = (L.solve(x - meanVec)).squaredNorm();
    return std::exp(-x.rows() * logSqrt2Pi - 0.5 * quadform) / L.determinant();
}
