#include "pointcloud.h"
#include <pcl/visualization/cloud_viewer.h>

using PointT = pcl::PointXYZRGB;
using PtCloud = pcl::PointCloud<PointT>;

PointCloud::Ptr PointCloud::create(const std::string &filePath) {
    std::ifstream fin(filePath, std::ios::in | std::ios::binary);
    if (!fin.is_open()) {
        std::cerr << "Cannot open file " + filePath + '\n';
        return nullptr;
    }

    Ptr pointCloud = Ptr(new PointCloud);
    float r, xyz[3];
    int count = 0;
    while (!fin.eof()) {
        fin.read((char *) &xyz, 3 * sizeof(float));
        fin.read((char *) &r, sizeof(float));

        Eigen::VectorXd tmp_xyz(3);
        tmp_xyz << xyz[0], xyz[1], xyz[2];
        pointCloud->xyz.push_back(tmp_xyz);

        ++count;
    }
    fin.close();

    std::cout << "Read " << count << " points from KITTI dataset\n";
    return pointCloud;
}

void PointCloud::visualize(double pointSize) {
    if (xyz.empty()) return;

    // 格式转换为pcl点云
    PtCloud::Ptr cloud = PtCloud::Ptr(new PtCloud);
    for (int i = 0; i < xyz.size(); ++i) {
        PointT pt;
        pt.x = xyz[i].x();
        pt.y = xyz[i].y();
        pt.z = xyz[i].z();
        pt.r = pt.g = pt.b = 255;
        cloud->push_back(pt);
    }

    // 可视化
    pcl::visualization::PCLVisualizer::Ptr viewer(new pcl::visualization::PCLVisualizer("3D Viewer"));
    viewer->addPointCloud<PointT>(cloud, "point cloud");
    viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, pointSize, "point cloud"); // 设置点云显示大小
    viewer->spin();
}

void PointCloud::visualize(const std::vector<int> &label, double pointSize) {
    if (xyz.size() != label.size()) return;

    // 统计总共多少类别
    int maxLabel = 0;
    for (int i = 0; i < label.size(); ++i) {
        if (label[i] > maxLabel) maxLabel = label[i];
    }
    int numClass = maxLabel + 1;

    // 生成colormap
    srand(time(NULL));
    std::vector<std::vector<int>> label_colormap(numClass + 1, std::vector<int>(3, 0));
    for (int i = 0; i < numClass; ++i) {
        label_colormap[i][0] = rand() % 255;
        label_colormap[i][1] = rand() % 255;
        label_colormap[i][2] = rand() % 255;
    }
    label_colormap[0][0] = label_colormap[0][1] = label_colormap[0][2] = 255;

    // 格式转换为pcl点云
    PtCloud::Ptr cloud = PtCloud::Ptr(new PtCloud);
    for (int i = 0; i < xyz.size(); ++i) {
        PointT pt;
        pt.x = xyz[i].x();
        pt.y = xyz[i].y();
        pt.z = xyz[i].z();

        auto rgb = label_colormap[label[i] + 1];
        pt.r = rgb[0];
        pt.g = rgb[1];
        pt.b = rgb[2];
        cloud->push_back(pt);
    }

    // 可视化
    pcl::visualization::PCLVisualizer::Ptr viewer(new pcl::visualization::PCLVisualizer("3D Viewer"));
    viewer->addPointCloud<PointT>(cloud, "point cloud");
    viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, pointSize, "point cloud"); // 设置点云显示大小
    viewer->spin();
}