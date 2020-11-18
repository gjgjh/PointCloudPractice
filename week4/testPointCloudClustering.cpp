#include "src/utils.h"
#include "src/groundextraction.h"
#include "src/pointcloudcluster.h"

int main() {
    // 读取数据
    std::string datapath = "/Users/huiguo/Code/PointCloudPractice/week4/data/005374.bin";
    auto cloud = readVelodyneBin(datapath);

    // 地面提取
    GroundExtraction groundExtraction(cloud, 3000, 100, 0.1);
    groundExtraction.startExtraction();
    auto non_ground = groundExtraction.getNonGround();

    // 点云聚类
    PointCloudCluster pointCloudCluster(non_ground, 1, 50, 25000);
    auto res = pointCloudCluster.cluster();

    // 可视化
    visualize(res, 2);

    return 0;
}
