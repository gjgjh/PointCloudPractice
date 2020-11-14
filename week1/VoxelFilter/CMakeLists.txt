cmake_minimum_required(VERSION 3.5)
project(VoxelFilter)

set(CMAKE_CXX_FLAGS "-std=c++11")
set(CMAKE_BUILD_TYPE "Debug")
set(EXECUTABLE_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/bin)

############### dependencies ######################
# eigen
find_package(Eigen3 3.3 REQUIRED)

# pcl
find_package(PCL REQUIRED COMPONENT common io visualization)
include_directories(${PCL_INCLUDE_DIRS})
add_definitions(${PCL_DEFINITIONS})

#################### source  ######################
add_library(voxelfilter
        src/pointcloud.cpp
        src/voxelfilter.cpp)
target_link_libraries(voxelfilter Eigen3::Eigen)

#################### test  ######################
add_executable(testVoxelFilter testVoxelFilter.cpp)
target_link_libraries(testVoxelFilter ${PCL_LIBRARIES} voxelfilter)