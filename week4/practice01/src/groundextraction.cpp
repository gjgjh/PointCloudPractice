#include "groundextraction.h"


void GroundExtraction::startExtraction() {
    // 设置RANSAC参数
    pcl::SACSegmentation<PointT> seg;
    seg.setOptimizeCoefficients(true);
    seg.setModelType(pcl::SACMODEL_PLANE);
    seg.setMethodType(pcl::SAC_RANSAC);
    seg.setMaxIterations(maxIter);
    seg.setDistanceThreshold(distThreshold);

    // 平面检测
    pcl::PointIndices::Ptr inliers(new pcl::PointIndices);
    pcl::ModelCoefficients::Ptr coefficients(new pcl::ModelCoefficients);

    ground = PtCloud::Ptr(new PtCloud());
    non_ground = data;
    int iter = 0;
    while (true) {
        seg.setInputCloud(non_ground);
        seg.segment(*inliers, *coefficients);

        // 判断是否能检测出平面。根据点数和法向判断
        auto coef = coefficients->values;
        double normDiff = coef[0] * coef[0] + coef[1] * coef[1] + (coef[2] - 1) * (coef[2] - 1);
        cout << "Ground extraction iter " << iter++ << ": "
             << inliers->indices.size() << " points, normDiff: " << normDiff << '\n';
        if (inliers->indices.size() < minGroundPointNum || normDiff > 0.01) {
            cout << "Ground points num: " << ground->size() << '\n';
            return;
        }

        // 保存地面和非地面
        pcl::ExtractIndices<PointT> extract;
        extract.setInputCloud(non_ground);
        extract.setIndices(inliers);

        auto ground_temp = PtCloud::Ptr(new PtCloud());
        non_ground = PtCloud::Ptr(new PtCloud());
        extract.setNegative(false);
        extract.filter(*ground_temp);
        extract.setNegative(true);
        extract.filter(*non_ground);

        *ground += *ground_temp;
    }
}

const PtCloud::Ptr &GroundExtraction::getNonGround() const {
    return non_ground;
}

const PtCloud::Ptr &GroundExtraction::getGround() const {
    return ground;
}
