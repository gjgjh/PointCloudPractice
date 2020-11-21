#include "src/pointcloud.h"
#include "src/groundextraction.h"

int main() {
    // 读取数据
    std::string datapath = "/Users/huiguo/Code/PointCloudPractice/week4/practice01/data/007022.bin";
    PointCloud::Ptr cloud = PointCloud::create(datapath);

    // 地面提取
    GroundExtraction groundExtraction(cloud, 3000, 100, 0.1);
    groundExtraction.startExtraction();
    auto non_ground = groundExtraction.getNonGround();
    auto ground = groundExtraction.getGround();

    // 可视化
    cloud->visualize();
    non_ground->visualize();
    ground->visualize();

    return 0;
}