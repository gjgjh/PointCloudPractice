#include "utils.h"


PtCloud::Ptr readVelodyneBin(const std::string &filepath) {
    std::ifstream fin(filepath, ios::in | ios::binary);
    if (!fin.is_open()) {
        std::cerr << "Cannot open file " + filepath + '\n';
        return nullptr;
    }

    PtCloud::Ptr cloud(new PtCloud);
    float r, xyz[3];
    while (!fin.eof()) {
        PointT pt;
        fin.read((char *) &xyz, 3 * sizeof(float));
        fin.read((char *) &r, sizeof(float));
        pt.x = xyz[0];
        pt.y = xyz[1];
        pt.z = xyz[2];
        pt.r = pt.g = pt.b = 255;
        cloud->push_back(pt);
    }
    fin.close();

    std::cout << "Read " << cloud->size() << " points from KITTI dataset\n";
    return cloud;
}

void visualize(const PtCloud::Ptr &ptCloud, double pointSize) {
    if (ptCloud == nullptr) return;

    pcl::visualization::PCLVisualizer::Ptr viewer(new pcl::visualization::PCLVisualizer("3D Viewer"));
    viewer->addPointCloud<PointT>(ptCloud, "point cloud");
    viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, pointSize, "point cloud"); // 设置点云显示大小
    viewer->spin();
}