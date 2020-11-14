#include "voxelfilter.h"

PointCloud::Ptr VoxelFilter::filter() {
    // 计算voxel采样维数
    Eigen::VectorXd min_b = pointCloud->min_b;
    Eigen::VectorXd max_b = pointCloud->max_b;

    dx = static_cast<long long>((max_b.x() - min_b.x()) / voxel_size.x());
    dy = static_cast<long long>((max_b.y() - min_b.y()) / voxel_size.y());
    dz = static_cast<long long>((max_b.z() - min_b.z()) / voxel_size.z());

    // 检查点云是否范围太大或voxel_size是否设置太小
    if (dx < 0 || dy < 0 || dz < 0 || (dx * dy * dz) > static_cast<long long>(std::numeric_limits<int>::max())) {
        std::cout << "Leaf size is too small for the input dataset. Integer indices would overflow\n";
        return nullptr;
    }

    // First pass: 计算每个点云对应的voxel index
    std::vector<CloudidxVoxelidx> index_vector;
    calVoxelIndex(index_vector);

    // Second pass: 按voxel index从小到大排序
    std::sort(index_vector.begin(), index_vector.end(), [](const CloudidxVoxelidx &lhs, const CloudidxVoxelidx &rhs) -> bool {
        return lhs.second < rhs.second;
    });

    // Third pass: 计算centroid，并保存结果
    PointCloud::Ptr output = PointCloud::Ptr(new PointCloud);
    calCentroid(index_vector, output);

    return output;
}

void VoxelFilter::calVoxelIndex(std::vector<CloudidxVoxelidx> &index_vector) {
    Eigen::VectorXd min_b = pointCloud->min_b;
    Eigen::VectorXd max_b = pointCloud->max_b;

    index_vector.reserve(pointCloud->xyz.size());
    for (int i = 0; i < pointCloud->xyz.size(); ++i) {
        double x = pointCloud->xyz[i].x(), y = pointCloud->xyz[i].y(), z = pointCloud->xyz[i].z();

        long long hx = std::floor(1. / voxel_size.x() * (x - min_b.x()));
        long long hy = std::floor(1. / voxel_size.y() * (y - min_b.y()));
        long long hz = std::floor(1. / voxel_size.z() * (z - min_b.z()));
        long long voxel_idx = hx + hy * dx + hz * dx * dy;

        index_vector.emplace_back(i, voxel_idx);
    }
}

void VoxelFilter::calCentroid(const std::vector<CloudidxVoxelidx> &index_vector, PointCloud::Ptr &output) {
    if (method == METHOD::CENTROID) {
        for (int i = 0; i < index_vector.size(); ++i) {
            Eigen::VectorXd centroid = Eigen::VectorXd::Zero(3, 1);
            centroid += pointCloud->xyz[index_vector[i].first];
            int count = 1;

            while (i + 1 < index_vector.size() && index_vector[i + 1].second == index_vector[i].second) { // 判断下一个点是否和当前voxel_index相同
                ++i;
                ++count;
                centroid += pointCloud->xyz[index_vector[i].first];
            }
            centroid /= count;

            output->xyz.push_back(centroid);
        }
    } else if (method == METHOD::RANDOM) {
        srand(time(NULL));

        for (int i = 0; i < index_vector.size(); ++i) {
            std::vector<int> tempIdx;
            tempIdx.push_back(index_vector[i].first);

            while (i + 1 < index_vector.size() && index_vector[i + 1].second == index_vector[i].second) { // 判断下一个点是否和当前voxel_index相同
                ++i;
                tempIdx.push_back(index_vector[i].first);
            }

            int randomIdx = tempIdx[rand() % tempIdx.size()];
            Eigen::VectorXd centroid = pointCloud->xyz[randomIdx];

            output->xyz.push_back(centroid);
        }
    }
}
