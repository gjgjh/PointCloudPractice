#include <string>

#include <pcl/visualization/cloud_viewer.h>

#include "src/pointcloud.h"
#include "src/pca.h"

int main() {
    // 加载点云数据
    std::string filePath = "/Users/huiguo/Code/PointCloudPractice/week1/PCA/data/airplane_0001.txt";
    PointCloud pointCloud = PointCloud::create(filePath);
    int number = pointCloud.xyz.size();

    // PCA分析
    PCA pca_analysis(pointCloud.xyz);

    typedef pcl::PointXYZ PointT;
    typedef pcl::PointCloud<PointT> PtCloud;
    PtCloud::Ptr output(new PtCloud());
    for (int i = 0; i < number; ++i) {
        PointT pt;
        pt.x = pointCloud.xyz[i].x();
        pt.y = pointCloud.xyz[i].y();
        pt.z = pointCloud.xyz[i].z();
        output->push_back(pt);
    }

    // 可视化三个主向量方向
    pcl::visualization::PCLVisualizer::Ptr viewer(new pcl::visualization::PCLVisualizer("3D Viewer"));
    viewer->addPointCloud<pcl::PointXYZ>(output, "point cloud");
    PointT pt(0, 0, 0);
    PointT vector1(pca_analysis.eigenvectors.col(0).x(), pca_analysis.eigenvectors.col(0).y(), pca_analysis.eigenvectors.col(0).z());
    PointT vector2(pca_analysis.eigenvectors.col(1).x(), pca_analysis.eigenvectors.col(1).y(), pca_analysis.eigenvectors.col(1).z());
    PointT vector3(pca_analysis.eigenvectors.col(2).x(), pca_analysis.eigenvectors.col(2).y(), pca_analysis.eigenvectors.col(2).z());
    viewer->addArrow(vector1, pt, 255, 0, 0, false, "Eigen Vector 1");
    viewer->addArrow(vector2, pt, 0, 255, 0, false, "Eigen Vector 2");
    viewer->addArrow(vector3, pt, 0, 0, 255, false, "Eigen Vector 3");
    viewer->spin();

    return 0;
}
