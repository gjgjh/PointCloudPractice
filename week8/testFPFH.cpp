#include "src/fpfh.h"
#include "src/utils.h"

int main() {
    // 加载点云数据
    std::string filePath = "/Users/huiguo/Code/PointCloudPractice/week7/data/airplane_0001.txt";
    pcl::PointCloud<PointT>::Ptr cloud;
    pcl::PointCloud<NormalT>::Ptr normal;
    utils::readDataset(filePath, cloud, normal);

    // 特征点描述
    FPFHEstimation::Ptr fpfh = FPFHEstimation::create(11, 11, 11, 0.1);
    int idx1 = 4564, idx2 = 3862;
    auto descriptor1 = fpfh->computeFPFH(cloud, normal, idx1);
    auto descriptor2 = fpfh->computeFPFH(cloud, normal, idx2);

    // 可视化并保存结果
    utils::writeDescriptor(descriptor1, "./descriptor1.txt");
    utils::visualizePoint(cloud, 3, idx1, 10);
    utils::writeDescriptor(descriptor2, "./descriptor2.txt");
    utils::visualizePoint(cloud, 3, idx2, 10);

    return 0;
}