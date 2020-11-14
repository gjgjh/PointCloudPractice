#ifndef POINTCLOUD_H
#define POINTCLOUD_H

#include "common.h"

class PointCloud{
public:
    typedef std::shared_ptr<PointCloud> Ptr;

    PointCloud()=default;

    /**
     * 工厂函数.
     * 务必确保文件路径正确，且文件符合格式：x y z normal_x normal_y normal_z
     * @param file_path 点云文件路径
     * @return 点云对象
     */
    static Ptr create(const std::string& filePath);

    std::vector<Eigen::VectorXd> xyz;       //!< 位置
    std::vector<Eigen::VectorXd> normal;    //!< 法向量

    Eigen::VectorXd min_b;                  //!< 点云bounding box范围下界：min_x, min_y, min_z
    Eigen::VectorXd max_b;                  //!< 点云bounding box范围上界：max_x, max_y, max_z
};

#endif // POINTCLOUD_H