#include <string>

#include <pcl/visualization/cloud_viewer.h>
#include <pcl/kdtree/kdtree_flann.h>

#include "src/pointcloud.h"
#include "src/pca.h"

/**
 * 根据给定的一组点云集合，计算法向量.
 * @param cloud 点云集合
 * @param normal 法向量 (n_x, n_y, n_z, curvature)
 */
void computePointNormal(const pcl::PointCloud<pcl::PointXYZ> &cloud, pcl::Normal &normal);

int main() {
    // 加载点云数据
    std::string filePath = "/Users/huiguo/Code/PointCloudPractice/week1/PCA/data/airplane_0001.txt";
    PointCloud pointCloud = PointCloud::create(filePath);
    int number = pointCloud.xyz.size();

    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>());
    for (int i = 0; i < number; ++i) {
        cloud->emplace_back(pointCloud.xyz[i].x(), pointCloud.xyz[i].y(), pointCloud.xyz[i].z());
    }

    // 构建KD-Tree
    pcl::KdTreeFLANN<pcl::PointXYZ> kdtree;
    kdtree.setInputCloud(cloud);

    // 遍历点云，并依据邻域点云计算法向量
    int K = 20;
    std::vector<int> pointIdxNKNSearch;
    std::vector<float> pointNKNSquaredDistance;
    pcl::PointCloud<pcl::Normal>::Ptr cloud_normals(new pcl::PointCloud<pcl::Normal>);

    for (int i = 0; i < cloud->size(); ++i) {
        pcl::PointXYZ searchPoint((*cloud)[i].x, (*cloud)[i].y, (*cloud)[i].z);
        if (kdtree.nearestKSearch(searchPoint, K, pointIdxNKNSearch, pointNKNSquaredDistance) > 0) {
            pcl::PointCloud<pcl::PointXYZ> neighbors;
            for (auto idx:pointIdxNKNSearch)
                neighbors.push_back((*cloud)[idx]);

            pcl::Normal normal;
            computePointNormal(neighbors, normal);
            cloud_normals->push_back(normal);
        } else {
            cloud_normals->push_back(pcl::Normal());
        }
    }

    // 可视化
    pcl::visualization::PCLVisualizer::Ptr viewer(new pcl::visualization::PCLVisualizer("3D Viewer"));
    viewer->addPointCloud<pcl::PointXYZ>(cloud, "point cloud");
    viewer->addPointCloudNormals<pcl::PointXYZ, pcl::Normal>(cloud, cloud_normals, 10, 0.05, "normals");
    viewer->spin();

    return 0;
}

void computePointNormal(const pcl::PointCloud<pcl::PointXYZ> &cloud, pcl::Normal &normal) {
    // 格式转换
    std::vector<Eigen::VectorXd> temp_cloud(cloud.size(), Eigen::VectorXd::Zero(3, 1));
    for (int i = 0; i < cloud.size(); ++i) {
        temp_cloud[i].x() = cloud[i].x;
        temp_cloud[i].y() = cloud[i].y;
        temp_cloud[i].z() = cloud[i].z;
    }

    // PCA分析
    PCA pca_analysis(temp_cloud);

    normal.normal_x = pca_analysis.eigenvectors.col(2).x();
    normal.normal_y = pca_analysis.eigenvectors.col(2).y();
    normal.normal_z = pca_analysis.eigenvectors.col(2).z();
    normal.curvature = pca_analysis.eigenvalues[2] / (pca_analysis.eigenvalues[0] + pca_analysis.eigenvalues[1] + pca_analysis.eigenvalues[2]);
}
