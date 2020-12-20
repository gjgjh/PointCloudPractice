#include "fpfh.h"

FPFHEstimation::Ptr FPFHEstimation::create(int numBins1, int numBins2, int numBins3, double radius) {
    FPFHEstimation::Ptr fpfh = std::make_shared<FPFHEstimation>();
    fpfh->numBins1 = numBins1;
    fpfh->numBins2 = numBins2;
    fpfh->numBins3 = numBins3;
    fpfh->searchRadius = radius;

    return fpfh;
}

std::vector<double> FPFHEstimation::computeFPFH(const pcl::PointCloud<PointT>::Ptr &cloud, const pcl::PointCloud<NormalT>::Ptr &normals, int idx) {
    descriptor.resize(0);

    // KNN搜索
    std::vector<int> indices;
    std::vector<float> dists2;
    kdtree.setInputCloud(cloud);
    if (kdtree.radiusSearch(idx, searchRadius, indices, dists2) == 0) return descriptor;

    // 计算每个点的SPFH TODO: 减少重复计算的情况
    std::vector<std::vector<double>> histogram;
    histogram.reserve(indices.size());
    for (int i = 0; i < indices.size(); ++i) {
        auto spfh = computeSPFH(cloud, normals, indices[i]);
        histogram.push_back(spfh);
    }

    // 加权
    descriptor = std::vector<double>(numBins1 + numBins2 + numBins3, 0);
    double weight, val_f1, val_f2, val_f3;
    double sum_f1 = 0.0, sum_f2 = 0.0, sum_f3 = 0.0;
    for (int i = 0; i < indices.size(); ++i) {
        if (indices[i] == idx) weight = 1.0;
        else weight = 1. / (dists2[i] * (indices.size() - 1));

        for (int j = 0; j < descriptor.size(); ++j) {
            if (0 <= j && j < numBins1) {
                val_f1 = weight * histogram[i][j];
                sum_f1 += val_f1;
                descriptor[j] += val_f1;
            } else if (numBins1 <= j && j < numBins1 + numBins2) {
                val_f2 = weight * histogram[i][j];
                sum_f2 += val_f2;
                descriptor[j] += val_f2;
            } else {
                val_f3 = weight * histogram[i][j];
                sum_f3 += val_f3;
                descriptor[j] += val_f3;
            }
        }
    }

    // 归一化
    for (int i = 0; i < descriptor.size(); ++i) {
        if (0 <= i && i < numBins1) {
            descriptor[i] = descriptor[i] / sum_f1;
        } else if (numBins1 <= i && i < numBins1 + numBins2) {
            descriptor[i] = descriptor[i] / sum_f2;
        } else {
            descriptor[i] = descriptor[i] / sum_f3;
        }
    }

    return descriptor;
}

std::vector<double> FPFHEstimation::computeSPFH(const pcl::PointCloud<PointT>::Ptr &cloud, const pcl::PointCloud<NormalT>::Ptr &normals, int idx) {
    // KNN搜索
    std::vector<int> indices;
    std::vector<float> dists2;
    if (kdtree.radiusSearch(idx, searchRadius, indices, dists2) == 0) return {};

    // 计算自己和邻居间的pair feature
    double incr = 1.0 / static_cast<double>(indices.size() - 1);
    std::vector<double> spfh(numBins1 + numBins2 + numBins3, 0);
    for (int i = 0; i < indices.size(); ++i) {
        if (indices[i] == idx) continue;
        Quadruplet features;
        computeQuadruplet(cloud, normals, idx, indices[i], features);

        // 统计直方图
        int f1_index = static_cast<int>(std::floor(numBins1 * (features.f1 + M_PI) / (2 * M_PI)));
        int f2_index = static_cast<int>(std::floor(numBins2 * (features.f2 + 1) / 2.0));
        int f3_index = static_cast<int>(std::floor(numBins3 * (features.f3 + 1) / 2.0));

        spfh[f1_index] += incr;
        spfh[numBins1 + f2_index] += incr;
        spfh[numBins1 + numBins2 + f3_index] += incr;
    }

    return spfh;
}

// 参考 https://github.com/PointCloudLibrary/pcl/blob/master/features/src/pfh.cpp
void FPFHEstimation::computeQuadruplet(const pcl::PointCloud<PointT>::Ptr &cloud, const pcl::PointCloud<NormalT>::Ptr &normals, int p_idx, int q_idx, Quadruplet &quadruplet) {
    // 格式转换
    PointT pt1 = cloud->points[p_idx];
    PointT pt2 = cloud->points[q_idx];
    NormalT normal1 = normals->points[p_idx];
    NormalT normal2 = normals->points[q_idx];
    Eigen::Vector4f p1(pt1.x, pt1.y, pt1.z, 0);
    Eigen::Vector4f p2(pt2.x, pt2.y, pt2.z, 0);
    Eigen::Vector4f n1(normal1.normal_x, normal1.normal_y, normal1.normal_z, 0);
    Eigen::Vector4f n2(normal2.normal_x, normal2.normal_y, normal2.normal_z, 0);

    Eigen::Vector4f dp2p1 = p2 - p1;
    dp2p1[3] = 0.0f;
    quadruplet.f4 = dp2p1.norm();

    if (quadruplet.f4 == 0.0) {
        std::cout << "Euclidean distance between points is 0!\n";
        quadruplet.f1 = quadruplet.f2 = quadruplet.f3 = quadruplet.f4 = 0;
        return;
    }

    Eigen::Vector4f n1_copy = n1, n2_copy = n2;
    n1_copy[3] = n2_copy[3] = 0.0f;
    float angle1 = n1_copy.dot(dp2p1) / quadruplet.f4;

    // Make sure the same point is selected as 1 and 2 for each pair
    float angle2 = n2_copy.dot(dp2p1) / quadruplet.f4;
    if (std::acos(std::fabs(angle1)) > std::acos(std::fabs(angle2))) {
        // switch p1 and p2
        n1_copy = n2;
        n2_copy = n1;
        n1_copy[3] = n2_copy[3] = 0.0f;
        dp2p1 *= (-1);
        quadruplet.f3 = -angle2;
    } else
        quadruplet.f3 = angle1;

    // Create a Darboux frame coordinate system u-v-w
    // u = n1; v = (p_idx - q_idx) x u / || (p_idx - q_idx) x u ||; w = u x v
    Eigen::Vector4f v = dp2p1.cross3(n1_copy);
    v[3] = 0.0f;
    float v_norm = v.norm();
    if (v_norm == 0.0f) {
        std::cout << "Norm of Delta x U is 0!\n";
        quadruplet.f1 = quadruplet.f2 = quadruplet.f3 = quadruplet.f4 = 0;
        return;
    }
    // Normalize v
    v /= v_norm;

    Eigen::Vector4f w = n1_copy.cross3(v);
    // Do not have to normalize w - it is a unit vector by construction

    v[3] = 0.0f;
    quadruplet.f2 = v.dot(n2_copy);
    w[3] = 0.0f;
    // Compute f1 = arctan (w * n2, u * n2) i.e. angle of n2 in the x=u, y=w coordinate system
    quadruplet.f1 = std::atan2(w.dot(n2_copy), n1_copy.dot(n2_copy));
}