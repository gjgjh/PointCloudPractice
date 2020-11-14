#include "kmeans.h"

void Kmeans::fit(const std::vector<Eigen::VectorXd> &data) {
    // 检查输入有效性
    assert(data.size() > 0 && k <= data.size() && "Data not valid\n");
    numDimensions = data[0].size();
    for (auto &item:data) {
        if (item.size() != numDimensions) {
            std::cerr << "Data dimension not match\n";
            return;
        }
    }
    this->data = data;

    // 初始聚类
    initialCentroids();

    // 迭代计算
    bool converge = false;
    int iter = 0;
    while (!converge) {
        computeClusters();
        double diff = computeCentroids();

        if (diff < tolerance || iter == maxIter) converge = true;
        ++iter;
    }
}

std::vector<int> Kmeans::predict(const std::vector<Eigen::VectorXd> &data) {
    assert(!centroids.empty() && "Should call fit() first\n");

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
        double minDist = std::numeric_limits<double>::max();

        // 寻找最近的聚类中心
        for (int j = 0; j < k; ++j) {
            double dist = (centroids[j] - data[i]).norm();
            if (dist < minDist) {
                minDist = dist;
                clusterIdx = j;
            }
        }

        ret[i] = clusterIdx;
    }

    return ret;
}

void Kmeans::initialCentroids() {
    centroids = std::vector<Eigen::VectorXd>(k, Eigen::VectorXd::Zero(numDimensions, 1));

    srand(time(NULL));
    for (int i = 0; i < k; ++i) {
        int randomIdx = rand() % data.size();
        centroids[i] = data[randomIdx];
    }
}

double Kmeans::computeCentroids() {
    auto oldCentroids = centroids;
    centroids = std::vector<Eigen::VectorXd>(k, Eigen::VectorXd::Zero(numDimensions, 1));

    for (int i = 0; i < k; ++i) {
        int count = 0;
        for (int j = 0; j < clusters[i].size(); ++j) {
            int idx = clusters[i][j];
            centroids[i] += data[idx];
            ++count;
        }
        centroids[i] /= (count + 1e-5);
    }

    // 计算新中心和旧中心的差别
    double diff = 0;
    for (int i = 0; i < k; ++i) diff += (oldCentroids[i] - centroids[i]).norm();
    return diff;
}

void Kmeans::computeClusters() {
    clusters = std::vector<std::vector<int>>(k, std::vector<int>());

    for (int i = 0; i < data.size(); ++i) {
        int clusterIdx = 0;
        double minDist = std::numeric_limits<double>::max();

        // 寻找最近的聚类中心
        for (int j = 0; j < k; ++j) {
            double dist = (centroids[j] - data[i]).norm();
            if (dist < minDist) {
                minDist = dist;
                clusterIdx = j;
            }
        }

        clusters[clusterIdx].push_back(i);
    }
}
