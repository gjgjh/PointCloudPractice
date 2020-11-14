cmake_minimum_required(VERSION 3.5)
project(clustering)

set(CMAKE_CXX_FLAGS "-std=c++11")
set(CMAKE_BUILD_TYPE "Debug")
set(EXECUTABLE_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/bin)
set(LIBRARY_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/lib)

############### dependencies ######################
# eigen
find_package(Eigen3 3.3 REQUIRED)

# pybind11
find_package(pybind11 REQUIRED)

set(THIRD_PARTY_LIBS
        Eigen3::Eigen
        ${PROJECT_SOURCE_DIR}/Thirdparty/libnabo/build/libnabo.dylib)

#################### source  ######################
# original library
add_library(clustering
        src/kmeans.cpp
        src/gmm.cpp
        src/spectralcluster.cpp)
target_link_libraries(clustering ${THIRD_PARTY_LIBS})

# python binding
pybind11_add_module(pycluster src/pycluster.cpp)
target_link_libraries(pycluster PRIVATE clustering)