#include "pointcloudcluster.h"

PtCloud::Ptr PointCloudCluster::cluster() {
    // 为聚类算法创建KdTree
    pcl::search::KdTree<PointT>::Ptr tree(new pcl::search::KdTree<PointT>());
    tree->setInputCloud(data);

    // 聚类
    std::vector <pcl::PointIndices> cluster_indices;
    pcl::EuclideanClusterExtraction <PointT> ec;
    ec.setClusterTolerance(tolerance);
    ec.setMinClusterSize(min_cluster_size);
    ec.setMaxClusterSize(max_cluster_size);
    ec.setSearchMethod(tree);
    ec.setInputCloud(data);
    ec.extract(cluster_indices);

    // 为不同类点云赋值颜色
    srand(time(NULL));
    PtCloud::Ptr output = data;
    for (int i = 0; i < cluster_indices.size(); ++i) {
        int r = rand() % 255, g = rand() % 255, b = rand() % 255;
        for (int j = 0; j < cluster_indices[i].indices.size(); ++j) {
            int pointIdx = cluster_indices[i].indices[j];
            output->points[pointIdx].r = r;
            output->points[pointIdx].g = g;
            output->points[pointIdx].b = b;
        }
    }

    cout << "Number of clusters: " << cluster_indices.size() << '\n';
    return output;
}