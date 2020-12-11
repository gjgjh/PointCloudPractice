#include "iss.h"

ISSEstimator::ISSEstimator(const PointCloud::Ptr &pointCloud, double searchRadius, double nonMaxRadius, double gamma21, double gamma32, int minNeighbors, double minEigVal)
        : searchRadius(searchRadius),
          nonMaxRadius(nonMaxRadius),
          gamma_21(gamma21),
          gamma_32(gamma32),
          minNeighbors(minNeighbors),
          minEigVal(minEigVal) {
    // 检查输入
    assert(searchRadius > 0 && nonMaxRadius > 0 && gamma21 >= 0 && gamma21 <= 1 && gamma32 >= 0 && gamma32 <= 1 && minNeighbors > 0 && "Parameters invalid\n");

    // 数据格式转换
    numPoints = pointCloud->xyz.size();
    data = Eigen::MatrixXf::Zero(3, numPoints);
    for (int i = 0; i < numPoints; ++i) {
        data.col(i) = pointCloud->xyz[i];
    }

    // 建立KNN搜索
    nnSearch = Nabo::NNSearchF::createKDTreeLinearHeap(data);
}

ISSEstimator::~ISSEstimator() {
    delete nnSearch;
}

PointCloud::Ptr ISSEstimator::compute() {
    // 计算每个点的响应（Scatter Matrix特征值）
    std::vector<double> third_eigvals(numPoints, -1);
    for (int i = 0; i < numPoints; ++i) {
        Eigen::Matrix3f cov;
        getScatterMatrix(i, cov);

        Eigen::SelfAdjointEigenSolver<Eigen::Matrix3f> solver(cov);
        double eigval1 = solver.eigenvalues()[2];
        double eigval2 = solver.eigenvalues()[1];
        double eigval3 = solver.eigenvalues()[0];

        // 判断是否为特征点
        if (eigval2 / eigval1 < gamma_21 && eigval3 / eigval2 < gamma_32)
            third_eigvals[i] = eigval3;
    }

    // 非极大值抑制（NMS）
    std::vector<bool> feat_max(numPoints, false);
    for (int i = 0; i < numPoints; ++i) {
        if (third_eigvals[i] <= minEigVal) continue;

        // KNN搜索
        int numNeighbors = 100;
        Eigen::VectorXi neighborIdx(numNeighbors);
        Eigen::VectorXf dists2(numNeighbors);
        nnSearch->knn(data.col(i), neighborIdx, dists2, numNeighbors, 0, 0, nonMaxRadius);

        if (neighborIdx.size() < minNeighbors) continue;

        bool is_max = true;
        int count = 0;
        for (int j = 0; j < neighborIdx.size(); j++) {
            if (neighborIdx[j] == -1) continue;
            ++count;

            if (third_eigvals[i] < third_eigvals[neighborIdx[j]]) {
                is_max = false;
                break;
            }
        }

        if (is_max && count >= minNeighbors) feat_max[i] = true;
    }

    // 输出结果（没有计算法向量）
    PointCloud::Ptr output(new PointCloud);
    for (int i = 0; i < numPoints; ++i) {
        if (feat_max[i]) {
            output->xyz.push_back(data.col(i));
            output->normal.push_back(Eigen::VectorXf::Zero(3, 1));
        }
    }

    std::cout << output->xyz.size() << " keypoints detected\n";
    return output;
}

// TODO: 加入权重
void ISSEstimator::getScatterMatrix(int current_index, Eigen::Matrix3f &cov_m) {
    Eigen::VectorXf central_point = data.col(current_index);
    cov_m = Eigen::Matrix3f::Zero();

    // KNN搜索
    int numNeighbors = 100;
    Eigen::VectorXi neighborIdx(numNeighbors);
    Eigen::VectorXf dists2(numNeighbors);
    nnSearch->knn(central_point, neighborIdx, dists2, numNeighbors, 0, 0, searchRadius);

    if (neighborIdx.size() < minNeighbors) return;

    // 计算Scatter Matrix
    std::vector<double> cov(9, 0);
    for (int i = 0; i < neighborIdx.size(); ++i) {
        if (neighborIdx[i] == -1) continue;
        Eigen::VectorXf neigh_point = data.col(neighborIdx[i]);

        for (int i = 0; i < 3; i++)
            for (int j = 0; j < 3; j++)
                cov[i * 3 + j] += (neigh_point[i] - central_point[i]) * (neigh_point[j] - central_point[j]);
    }

    cov_m << cov[0], cov[1], cov[2],
            cov[3], cov[4], cov[5],
            cov[6], cov[7], cov[8];
    cov_m /= (neighborIdx.size() - 1);
}