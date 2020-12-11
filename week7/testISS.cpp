#include <string>

#include <pcl/visualization/cloud_viewer.h>

#include "src/iss.h"
#include "src/pointcloud.h"

typedef pcl::PointXYZ PointT;
typedef pcl::PointCloud<PointT> PtCloud;

PtCloud::Ptr pointCloud2pclCloud(const PointCloud::Ptr &cloud);

int main() {
    // 加载点云数据
    std::string filePath = "/Users/huiguo/Code/PointCloudPractice/week7/data/piano_0001.txt";
    PointCloud::Ptr pointCloud = PointCloud::create(filePath);

    // ISS关键点提取
    ISSEstimator issEstimator(pointCloud, 0.1, 0.1, 0.8, 0.8, 5, 0.0001);
    auto pointCloudKpts = issEstimator.compute();

    // 可视化点云和关键点
    pcl::visualization::PCLVisualizer::Ptr viewer(new pcl::visualization::PCLVisualizer("3D Viewer"));
    viewer->addPointCloud<pcl::PointXYZ>(pointCloud2pclCloud(pointCloud), "point cloud");
    viewer->addPointCloud<pcl::PointXYZ>(pointCloud2pclCloud(pointCloudKpts), "keypoints");
    viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 10, "keypoints");
    viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_COLOR, 0, 1, 0, "keypoints");
    viewer->spin();

    return 0;
}

PtCloud::Ptr pointCloud2pclCloud(const PointCloud::Ptr &cloud) {
    PtCloud::Ptr output(new PtCloud());
    for (int i = 0; i < cloud->xyz.size(); ++i) {
        PointT pt;
        pt.x = cloud->xyz[i].x();
        pt.y = cloud->xyz[i].y();
        pt.z = cloud->xyz[i].z();
        output->push_back(pt);
    }

    return output;
}
