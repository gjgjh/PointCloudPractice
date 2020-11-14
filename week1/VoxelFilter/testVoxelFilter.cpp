#include <string>

#include <pcl/visualization/cloud_viewer.h>

#include "src/pointcloud.h"
#include "src/voxelfilter.h"

int main() {
    // 加载点云数据
    std::string filePath = "/Users/huiguo/Code/PointCloudPractice/week1/VoxelFilter/data/car_0001.txt";
    PointCloud::Ptr pointCloud = PointCloud::create(filePath);
    int number = pointCloud->xyz.size();

    typedef pcl::PointXYZ PointT;
    typedef pcl::PointCloud<PointT> PtCloud;

    PtCloud::Ptr cloud_before(new PtCloud());
    for (int i = 0; i < number; ++i) {
        PointT pt;
        pt.x = pointCloud->xyz[i].x();
        pt.y = pointCloud->xyz[i].y();
        pt.z = pointCloud->xyz[i].z();
        cloud_before->push_back(pt);
    }

   // // 可视化处理前点云
   // pcl::visualization::PCLVisualizer::Ptr viewer(new pcl::visualization::PCLVisualizer("3D Viewer"));
   // viewer->addPointCloud<pcl::PointXYZ>(cloud_before, "point cloud before");
   // viewer->spin();

    // 体素滤波
    // VoxelFilter voxelFilter(pointCloud, {0.08, 0.08, 0.08}, VoxelFilter::METHOD::RANDOM);
    VoxelFilter voxelFilter(pointCloud, {0.08, 0.08, 0.08}, VoxelFilter::METHOD::CENTROID);
    auto filtered = voxelFilter.filter();

    std::cout << "Before filtering: " << pointCloud->xyz.size() << " points\n";
    std::cout << "After filtering: " << filtered->xyz.size() << " points\n";

    // 可视化滤波后点云
    PtCloud::Ptr cloud_after(new PtCloud());
    for (int i = 0; i < filtered->xyz.size(); ++i) {
        PointT pt;
        pt.x = filtered->xyz[i].x();
        pt.y = filtered->xyz[i].y();
        pt.z = filtered->xyz[i].z();
        cloud_after->push_back(pt);
    }

    pcl::visualization::PCLVisualizer::Ptr viewer(new pcl::visualization::PCLVisualizer("3D Viewer"));
    viewer->addPointCloud<pcl::PointXYZ>(cloud_after, "point cloud after");
    viewer->spin();

    return 0;
}
