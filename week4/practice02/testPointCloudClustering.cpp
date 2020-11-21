#include "src/pointcloud.h"
#include "src/groundextraction.h"
#include "src/pointcloudcluster.h"

int main() {
    // 读取数据
    std::string datapath = "/Users/huiguo/Code/PointCloudPractice/week4/practice01/data/007022.bin";
    PointCloud::Ptr cloud = PointCloud::create(datapath);

    // 地面提取
    GroundExtraction groundExtraction(cloud, 3000, 100, 0.1);
    groundExtraction.startExtraction();
    auto non_ground = groundExtraction.getNonGround();

    // 点云聚类
    PointCloudCluster pointCloudCluster(non_ground, 0.5, 50, 25000);
    auto labels = pointCloudCluster.cluster();

    // 可视化
    non_ground->visualize(labels, 2);

    return 0;
}