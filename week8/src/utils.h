#ifndef UTILS_H
#define UTILS_H

#include "common.h"

namespace utils {

/**
 * 读取ModelNet40数据集
 * @param filePath 文件路径
 * @param cloud 读取后的点云数据
 * @param normals 读取后的法向量数据
 */
void readDataset(const std::string &filePath, pcl::PointCloud<PointT>::Ptr &cloud, pcl::PointCloud<NormalT>::Ptr &normals);

/**
 * 可视化点云
 * @param cloud 点云数据
 * @param cloudSize 点云中点的可视化大小
 */
void visualize(const pcl::PointCloud<PointT>::Ptr &cloud, int cloudSize);

/**
 * 可视化点云和一个特定的点
 * @param cloud 点云数据
 * @param cloudSize 点云中点的可视化大小
 * @param pointIdx 该特定点的index。必须在点云范围内
 * @param pointSize 该特定点的可视化大小
 */
void visualizePoint(const pcl::PointCloud<PointT>::Ptr &cloud, int cloudSize, int pointIdx, int pointSize);

/**
 * PCL viewer选点回调函数。按下shift键并选点即可输出该点信息
 * @param event 选点事件
 * @param viewer_void PCL Viewer指针
 */
void pointPickingEventOccurred(const pcl::visualization::PointPickingEvent &event, void *viewer_void);

/**
 * 将描述向量输出为本地txt文件
 * @param descriptor 描述向量
 * @param filePath 文件路径
 */
void writeDescriptor(const std::vector<double> descriptor, const std::string &filePath);

}
#endif // UTILS_H