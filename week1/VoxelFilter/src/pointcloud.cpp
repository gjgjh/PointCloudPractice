#include "pointcloud.h"

PointCloud::Ptr PointCloud::create(const std::string &filePath) {
    std::ifstream fin(filePath);
    if (!fin.is_open()) {
        std::cerr << "Error when opening file: " << filePath;
        return nullptr;
    }

    double x, y, z, normal_x, normal_y, normal_z;
    char sep;
    int count = 0;
    Ptr pointCloud = Ptr(new PointCloud);

    while (fin >> x >> sep >> y >> sep >> z >> sep >> normal_x >> sep >> normal_y >> sep >> normal_z) {
        Eigen::VectorXd tmp_xyz(3), tmp_normal(3);
        tmp_xyz << x, y, z;
        tmp_normal << normal_x, normal_y, normal_z;
        pointCloud->xyz.push_back(tmp_xyz);
        pointCloud->normal.push_back(tmp_normal);
        ++count;

        // 计算bounding box
        if (count == 1) {
            pointCloud->min_b = tmp_xyz;
            pointCloud->max_b = tmp_xyz;
        }

        pointCloud->min_b.x() = std::min(pointCloud->min_b.x(), x);
        pointCloud->min_b.y() = std::min(pointCloud->min_b.y(), y);
        pointCloud->min_b.z() = std::min(pointCloud->min_b.z(), z);
        pointCloud->max_b.x() = std::max(pointCloud->max_b.x(), x);
        pointCloud->max_b.y() = std::max(pointCloud->max_b.y(), y);
        pointCloud->max_b.z() = std::max(pointCloud->max_b.z(), z);
    }
    std::cout << count << " points loaded\n";
    fin.close();

    return pointCloud;
}