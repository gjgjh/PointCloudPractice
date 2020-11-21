cmake_minimum_required(VERSION 3.5)
project(PointCloudDetection)

set(CMAKE_CXX_FLAGS "-std=c++11")
set(CMAKE_BUILD_TYPE "Debug")
set(EXECUTABLE_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/bin)
set(LIBRARY_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/lib)

################## dependencies ###################
# eigen
find_package(Eigen3 3.3 REQUIRED)

# pcl
find_package(PCL REQUIRED COMPONENT common io visualization)
include_directories(${PCL_INCLUDE_DIRS})
add_definitions(${PCL_DEFINITIONS})

set(THIRD_PARTY_LIBS
        Eigen3::Eigen
        ${PCL_LIBRARIES}
        ${PROJECT_SOURCE_DIR}/Thirdparty/libnabo/build/libnabo.dylib)

#################### source #######################
add_library(mylib
        src/pointcloud.cpp
        src/ransac.cpp
        src/ransac_types.cpp
        src/groundextraction.cpp
        src/pointcloudcluster.cpp)
target_link_libraries(mylib ${THIRD_PARTY_LIBS})

#################### test #########################
add_executable(testGroundPointRemoval testGroundPointRemoval.cpp)
target_link_libraries(testGroundPointRemoval mylib)

add_executable(testPointCloudClustering testPointCloudClustering.cpp)
target_link_libraries(testPointCloudClustering mylib)
