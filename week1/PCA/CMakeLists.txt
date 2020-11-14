cmake_minimum_required(VERSION 3.5)
project(myPCA)

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
add_library(mypca
        src/pointcloud.cpp
        src/pca.cpp)
target_link_libraries(mypca Eigen3::Eigen)

#################### test  ######################
add_executable(testPCA testPCA.cpp)
target_link_libraries(testPCA ${PCL_LIBRARIES} mypca)

add_executable(testPCA2 testPCA2.cpp)
target_link_libraries(testPCA2 ${PCL_LIBRARIES} mypca)

add_executable(testPCA3 testPCA3.cpp)
target_link_libraries(testPCA3 ${PCL_LIBRARIES} mypca)