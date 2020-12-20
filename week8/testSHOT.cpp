#include "src/fpfh.h"
#include "src/utils.h"

#include <pcl/features/shot.h>

int main() {
    // 加载点云数据
    std::string filePath = "/Users/huiguo/Code/PointCloudPractice/week7/data/airplane_0001.txt";
    pcl::PointCloud<PointT>::Ptr cloud;
    pcl::PointCloud<NormalT>::Ptr normal;
    utils::readDataset(filePath, cloud, normal);

    // 特征点描述
    typedef pcl::SHOT352 ShotFeature;
    pcl::search::KdTree<pcl::PointXYZ>::Ptr tree(new pcl::search::KdTree<pcl::PointXYZ>);
    pcl::PointCloud<ShotFeature>::Ptr descriptors(new pcl::PointCloud<ShotFeature>);

    pcl::SHOTEstimation<pcl::PointXYZ, pcl::Normal, ShotFeature> shot;
    shot.setInputCloud(cloud);
    shot.setInputNormals(normal);
    shot.setSearchMethod(tree);
    shot.setLRFRadius(0.1);
    shot.setRadiusSearch(0.1);
    shot.compute(*descriptors);

    std::cout << "SHOT output points.size(): " << descriptors->points.size() << std::endl;

    // 可视化并保存结果
    int idx1 = 4564, idx2 = 3862;
    ShotFeature descriptor1 = descriptors->points[idx1];
    ShotFeature descriptor2 = descriptors->points[idx2];
    std::vector<double> desp1(std::begin(descriptor1.descriptor), std::end(descriptor1.descriptor));
    std::vector<double> desp2(std::begin(descriptor2.descriptor), std::end(descriptor2.descriptor));

    utils::writeDescriptor(desp1, "./descriptor1.txt");
    utils::visualizePoint(cloud, 3, idx1, 10);
    utils::writeDescriptor(desp2, "./descriptor2.txt");
    utils::visualizePoint(cloud, 3, idx2, 10);

    return 0;
}