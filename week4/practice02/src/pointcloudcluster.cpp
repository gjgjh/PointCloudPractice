#include "pointcloudcluster.h"


PointCloudCluster::PointCloudCluster(const PointCloud::Ptr &cloud, double tolerance, int minClusterSize, int maxClusterSize) {
    this->tolerance = tolerance;
    this->min_cluster_size = minClusterSize;
    this->max_cluster_size = maxClusterSize;

    // 检查数据有效性，并进行数据格式转换
    assert(cloud->xyz.size() > 0 && "Data not valid\n");
    numPoints = cloud->xyz.size();
    numDimensions = cloud->xyz[0].size();
    data = Eigen::MatrixXf::Zero(numDimensions, numPoints);
    for (int i = 0; i < numPoints; ++i) {
        auto point = cloud->xyz[i];
        if (point.size() != numDimensions) {
            std::cerr << "Data dimension not match\n";
            return;
        } else {
            Eigen::VectorXf temp = point.cast<float>();
            data.col(i) = temp;
        }
    }
}

PointCloudCluster::~PointCloudCluster() {
    delete nnSearch;
}

std::vector<int> PointCloudCluster::cluster() {
    // 建立Kd-Tree
    nnSearch = Nabo::NNSearchF::createKDTreeLinearHeap(data);

    int classLabel = 0;
    std::vector<int> labels(numPoints, -1);

    // 遍历每一个尚未处理过的点
    std::vector<bool> processed(numPoints, false);
    for (int i = 0; i < numPoints; ++i) {
        if (processed[i]) continue;

        // flood fill算法聚类
        std::vector<int> cluster_indices;
        dfs(i, processed, cluster_indices);

        // 判断类别是否满足条件
        if (min_cluster_size < cluster_indices.size() && cluster_indices.size() < max_cluster_size) {
            for (int j = 0; j < cluster_indices.size(); ++j) {
                int idx = cluster_indices[j];
                labels[idx] = classLabel;
            }
            ++classLabel;
        }
    }
    std::cout << classLabel << " classes found\n";

    return labels;
}

void PointCloudCluster::dfs(int index, std::vector<bool> &processed, std::vector<int> &indices) {
    processed[index] = true;
    indices.push_back(index);

    // KNN搜索
    int numNeighbors = 100;
    Eigen::VectorXi neighborIdx(numNeighbors);
    Eigen::VectorXf dists2(numNeighbors);
    nnSearch->knn(data.col(index), neighborIdx, dists2, numNeighbors, 0, 0, tolerance);

    for (int i = 0; i < numNeighbors; ++i) {
        if (neighborIdx[i] == -1 || processed[neighborIdx[i]]) continue;

        // 如果是邻居，且未被访问过，则DFS
        dfs(neighborIdx[i], processed, indices);
    }
}
