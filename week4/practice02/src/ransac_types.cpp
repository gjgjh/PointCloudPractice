#include "ransac_types.h"

RansacPlane3D::RansacPlane3D(const std::vector<Eigen::VectorXd> &data, double distThreshold, double probGoodSample, int maxIter) :
        Ransac(data, distThreshold, 3, probGoodSample, maxIter) {
    assert(numDimensions == 3 && data[0].size() == 3 && "Type not matched\n");
}

void RansacPlane3D::fit(const std::vector<Eigen::VectorXd> &data, Eigen::VectorXd &model) {
    assert(data.size() == 3 && data[0].size() == 3 && "Data not valid\n");

    auto point1 = data[0], point2 = data[1], point3 = data[2];
    auto vec1 = point2 - point1, vec2 = point3 - point1;
    auto normal = Eigen::Vector3d{vec1.x(), vec1.y(), vec1.z()}.cross(Eigen::Vector3d{vec2.x(), vec2.y(), vec2.z()});
    normal.normalize();
    double d = -point1.dot(normal);

    model = Eigen::VectorXd::Zero(4, 1);
    model << normal[0], normal[1], normal[2], d;

    double sign = normal[2] < 0 ? -1 : 1;   // 尽可能使z法方向为正
    model *= sign;
}

// TODO：优化随机样本的生成效率
std::vector<Eigen::VectorXd> RansacPlane3D::generateData() {
    int count = 0;
    while (count++ < maxIter) {
        // 随机产生数据
        std::vector<int> randomIdx;
        randomIdx.reserve(data.size());
        for (int i = 0; i < data.size(); ++i)
            randomIdx.push_back(i);
        std::random_shuffle(randomIdx.begin(), randomIdx.end());

        // 判断数据是否degenerate
        auto point1 = data[randomIdx[0]], point2 = data[randomIdx[1]], point3 = data[randomIdx[2]];
        auto vec1 = point2 - point1, vec2 = point3 - point1;
        auto normal = Eigen::Vector3d{vec1.x(), vec1.y(), vec1.z()}.cross(Eigen::Vector3d{vec2.x(), vec2.y(), vec2.z()});

        if (normal == Eigen::Vector3d::Zero()) continue;
        else return {point1, point2, point3};
    }

    return std::vector<Eigen::VectorXd>();
}

double RansacPlane3D::calDist(const Eigen::VectorXd &data, const Eigen::VectorXd &model) {
    double normal_x = model[0], normal_y = model[1], normal_z = model[2], d = model[3];

    double dist = abs(normal_x * data.x() + normal_y * data.y() + normal_z * data.z() + d);
    dist /= sqrt(normal_x * normal_x + normal_y * normal_y + normal_z * normal_z);

    return dist;
}
