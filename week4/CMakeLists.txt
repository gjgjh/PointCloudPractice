cmake_minimum_required(VERSION 3.5)
project(PointCloudDetection)

set(CMAKE_CXX_FLAGS "-std=c++11")
set(CMAKE_BUILD_TYPE "Debug")
set(EXECUTABLE_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/bin)
set(LIBRARY_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/lib)

################## dependencies ###################
# pcl
find_package(PCL REQUIRED COMPONENT common io visualization segmentation)
include_directories(${PCL_INCLUDE_DIRS})
add_definitions(${PCL_DEFINITIONS})

#################### source #######################
add_library(mylib
        src/utils.cpp
        src/groundextraction.cpp
        src/pointcloudcluster.cpp)
target_link_libraries(mylib ${PCL_LIBRARIES})

#################### test #########################
add_executable(testGroundPointRemoval testGroundPointRemoval.cpp)
target_link_libraries(testGroundPointRemoval mylib)

add_executable(testPointCloudClustering testPointCloudClustering.cpp)
target_link_libraries(testPointCloudClustering mylib)
