cmake_minimum_required(VERSION 3.5)
project(ISSEstimator)

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

set(THIRD_PARTY_LIBS
        Eigen3::Eigen
        ${PROJECT_SOURCE_DIR}/Thirdparty/libnabo/build/libnabo.dylib)

#################### source  ######################
add_library(libiss
        src/pointcloud.cpp
        src/iss.cpp)
target_link_libraries(libiss ${THIRD_PARTY_LIBS})

#################### test  ######################
add_executable(testISS testISS.cpp)
target_link_libraries(testISS ${PCL_LIBRARIES} libiss)