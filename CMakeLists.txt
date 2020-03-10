SET(PROJECT_NAME POSE_GRAPH)

PROJECT(${PROJECT_NAME})
cmake_minimum_required(VERSION 3.5.1)

if(COMPILER_SUPPORTS_CXX17)
    message(STATUS "Using C++ 17")
    add_compile_options(-std=c++17)
elseif(COMPILER_SUPPORTS_CXX11)
    message(STATUS "Using C++ 11")
    add_compile_options(-std=c++11)
elseif(COMPILER_SUPPORTS_CXX0X)
    message(STATUS "Using C++ 0x")
    add_compile_options(-std=c++0x)
endif()

set(CMAKE_BUILD_TYPE Release)
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS}" )
LIST(APPEND CMAKE_MODULE_PATH ${PROJECT_SOURCE_DIR}/cmake)

# find packages
set(EIGEN_INCLUDE_DIR /usr/local/include/eigen3)
set(GTSAM_INCLUDE_DIR /usr/local/include/gtsam)
find_package(Eigen3 REQUIRED)
find_package(GTSAM REQUIRED)

# add include directories
include_directories(
  ${PROJECT_SOURCE_DIR}/include/
  ${EIGEN_INCLUDE_DIR}
  ${GTSAM_INCLUDE_DIR}
) 

# add library and compile executable
add_library(data_loader_lib src/data_loader.cpp)
add_executable(data_loader src/data_loader.cpp)
target_link_libraries(data_loader data_loader_lib gtsam)
