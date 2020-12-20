cmake_minimum_required(VERSION 3.5)
project(descriptor)

set(CMAKE_CXX_FLAGS "-std=c++11")
set(CMAKE_BUILD_TYPE "Debug")
set(EXECUTABLE_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/bin)

############### dependencies ######################
# eigen
find_package(Eigen3 3.3 REQUIRED)

# pcl
find_package(PCL REQUIRED COMPONENT common io visualization features)
include_directories(${PCL_INCLUDE_DIRS})
add_definitions(${PCL_DEFINITIONS})

set(THIRD_PARTY_LIBS Eigen3::Eigen ${PCL_LIBRARIES})

#################### source  ######################
add_library(mylib
        src/utils.cpp
        src/fpfh.cpp)
target_link_libraries(mylib ${THIRD_PARTY_LIBS})

#################### test  ######################
add_executable(testFPFH testFPFH.cpp)
target_link_libraries(testFPFH mylib)

add_executable(testSHOT testSHOT.cpp)
target_link_libraries(testSHOT mylib)