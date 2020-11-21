#include "groundextraction.h"


void GroundExtraction::startExtraction() {
    std::vector<int> inliers;
    Eigen::VectorXd coefficients;
    ground = PointCloud::Ptr(new PointCloud());
    non_ground = data;
    int iter = 0;
    while (true) {
        // RANSAC
        RansacPlane3D ransac(non_ground->xyz, distThreshold, 0.99, maxIter);
        ransac.execute(inliers, coefficients);

        // 判断是否能检测出平面。根据点数和法向判断
        auto coef = coefficients;
        double normDiff = coef[0] * coef[0] + coef[1] * coef[1] + (coef[2] - 1) * (coef[2] - 1);
        std::cout << "Ground extraction iter " << iter++ << ": "
                  << inliers.size() << " points, normDiff: " << normDiff << '\n';
        if (inliers.size() < minGroundPointNum || normDiff > 0.1) {
            std::cout << "Ground points num: " << ground->xyz.size() << '\n';
            return;
        }

        // 保存地面点和非地面点
        auto non_ground_tmp = PointCloud::Ptr(new PointCloud());
        int j = 0;
        for (int i = 0; i < non_ground->xyz.size(); ++i) {
            if (j < inliers.size() && i == inliers[j]) { // 如果当前点是地面点
                ground->xyz.push_back(non_ground->xyz[i]);
                ++j;
            } else {
                non_ground_tmp->xyz.push_back(non_ground->xyz[i]);
            }
        }

        non_ground = non_ground_tmp;
    }
}

const PointCloud::Ptr &GroundExtraction::getNonGround() const {
    return non_ground;
}

const PointCloud::Ptr &GroundExtraction::getGround() const {
    return ground;
}
