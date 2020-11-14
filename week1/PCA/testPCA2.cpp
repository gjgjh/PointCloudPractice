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

    // 将点云投影为前两个主分量
    typedef pcl::PointXYZ PointT;
    typedef pcl::PointCloud<PointT> PtCloud;
    PtCloud::Ptr output(new PtCloud());
    for (int i = 0; i < number; ++i) {
        auto projected = pca_analysis.project(pointCloud.xyz[i]);

        PointT pt;
        pt.x = projected.x();
        pt.y = projected.y();
        pt.z = 0;
        output->push_back(pt);
    }

    // 可视化
    pcl::visualization::PCLVisualizer::Ptr viewer(new pcl::visualization::PCLVisualizer("3D Viewer"));
    viewer->addPointCloud<pcl::PointXYZ>(output, "point cloud");
    viewer->spin();

    return 0;
}
