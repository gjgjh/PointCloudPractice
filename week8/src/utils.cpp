#include "utils.h"

namespace utils {

void readDataset(const std::string &filePath, pcl::PointCloud<PointT>::Ptr &cloud, pcl::PointCloud<NormalT>::Ptr &normals) {
    std::ifstream fin(filePath);
    if (!fin.is_open()) {
        std::cerr << "Error when opening file: " << filePath;
        return;
    }

    double x, y, z, normal_x, normal_y, normal_z;
    char sep;
    int count = 0;
    cloud = std::make_shared<pcl::PointCloud<PointT>>();
    normals = std::make_shared<pcl::PointCloud<NormalT>>();

    while (fin >> x >> sep >> y >> sep >> z >> sep >> normal_x >> sep >> normal_y >> sep >> normal_z) {
        PointT pt(x, y, z);
        cloud->push_back(pt);
        NormalT normal(normal_x, normal_y, normal_z);
        normals->push_back(normal);

        ++count;
    }
    std::cout << count << " points loaded\n";
    fin.close();
}

void visualize(const pcl::PointCloud<PointT>::Ptr &cloud, int size) {
    pcl::visualization::PCLVisualizer::Ptr viewer(new pcl::visualization::PCLVisualizer("3D Viewer"));
    viewer->addPointCloud<PointT>(cloud, "point cloud");
    viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, size, "point cloud");
    viewer->registerPointPickingCallback(pointPickingEventOccurred, (void *) &viewer);
    viewer->spin();
}

void visualizePoint(const pcl::PointCloud<PointT>::Ptr &cloud, int cloudSize, int pointIdx, int pointSize) {
    auto keypoint = std::make_shared<pcl::PointCloud<PointT>>();
    keypoint->push_back(cloud->points[pointIdx]);

    pcl::visualization::PCLVisualizer::Ptr viewer(new pcl::visualization::PCLVisualizer("3D Viewer"));
    viewer->addPointCloud<PointT>(cloud, "point cloud");
    viewer->addPointCloud<pcl::PointXYZ>(keypoint, "keypoint");
    viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, cloudSize, "point cloud");
    viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, pointSize, "keypoint");
    viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_COLOR, 0, 1, 0, "keypoint");
    viewer->registerPointPickingCallback(pointPickingEventOccurred, (void *) &viewer);
    viewer->spin();
}

void pointPickingEventOccurred(const pcl::visualization::PointPickingEvent &event, void *viewer_void) {
    std::cout << "[INFO] Point picking event occurred" << std::endl;

    float x, y, z;
    if (event.getPointIndex() == -1) {
        return;
    }
    event.getPoint(x, y, z);
    std::cout << "[INFO] Point index: " << event.getPointIndex() << ", coordinate ( " << x << ", " << y << ", " << z << ")" << std::endl;
}

void writeDescriptor(const std::vector<double> descriptor, const std::string &filePath) {
    std::ofstream fout;
    fout.open(filePath);

    for (int i = 0; i < descriptor.size(); ++i) {
        fout << descriptor[i] << '\n';
    }

    fout.close();
    std::cout << "Descriptor file saved at: " << filePath << '\n';
}

}
