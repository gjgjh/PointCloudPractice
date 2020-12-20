#ifndef COMMON_H
#define COMMON_H

// std
#include <vector>
#include <string>
#include <iostream>
#include <fstream>
#include <memory>

// Eigen
#include <Eigen/Core>
#include <Eigen/Dense>

// PCL
#include <pcl/visualization/cloud_viewer.h>
#include <pcl/kdtree/kdtree_flann.h>

typedef pcl::PointXYZ PointT;
typedef pcl::Normal NormalT;

#endif // COMMON_H