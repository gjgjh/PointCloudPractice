#ifndef POINTCLOUD_H
#define POINTCLOUD_H

#include "common.h"

class PointCloud {
public:
    typedef std::shared_ptr<PointCloud> Ptr;

    PointCloud() = default;

    /**
     * 工厂函数.
     * 务必确保文件路径正确，且文件符合KITTI激光点云格式
     * @param file_path 点云文件路径
     * @return 点云对象指针
     */
    static Ptr create(const std::string &filePath);

    /**
     * 利用PCL库可视化点云
     * @param pointSize 点云显示大小
     */
    void visualize(double pointSize = 1);

    /**
     * 利用PCL库可视化点云及label
     * @param label 点云对应label
     * @param pointSize 点云显示大小
     */
    void visualize(const std::vector<int> &label, double pointSize = 1);

    std::vector<Eigen::VectorXd> xyz;       //!< 位置
};

#endif // POINTCLOUD_H