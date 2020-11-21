#ifndef COMMON_H
#define COMMON_H

// std
#include <vector>
#include <string>
#include <iostream>
#include <fstream>

// pcl
#include <pcl/ModelCoefficients.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/kdtree/kdtree.h>
#include <pcl/visualization/cloud_viewer.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/segmentation/extract_clusters.h>

using PointT = pcl::PointXYZRGB;
using PtCloud = pcl::PointCloud<PointT>;

#endif // COMMON_H