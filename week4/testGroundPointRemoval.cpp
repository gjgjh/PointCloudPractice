#include "src/utils.h"
#include "src/groundextraction.h"

int main() {
    // 读取数据
    std::string datapath = "/Users/huiguo/Code/PointCloudPractice/week4/data/007022.bin";
    auto cloud = readVelodyneBin(datapath);

    // 地面提取
    GroundExtraction groundExtraction(cloud, 3000, 100, 0.1);
    groundExtraction.startExtraction();
    auto non_ground = groundExtraction.getNonGround();
    auto ground = groundExtraction.getGround();

    // 可视化
    visualize(cloud);
    visualize(non_ground);
    visualize(ground);

    return 0;
}