cmake_minimum_required(VERSION 3.5)
project(geometry_benchmark)
set(CMAKE_BUILD_TYPE Release)
set(CMAKE_CXX_STANDARD 11)

find_package(Eigen3 REQUIRED)

#find_package(Boost REQUIRED fil)


#message("mrpt LIB = ${MRPT_LIBRARIES}")
message("benchmark LIB = ${benchmark_LIBRARIES}")

include_directories(
  include
)

#add_executable(pose3_benchmark
#  pose3_benchmark.cpp)
#target_link_libraries(pose3_benchmark
#  gtsam
#  benchmark::benchmark)

add_library(google_transform
  src/rigid_transform.cc
  src/transform.cc
  src/pose3d.cpp)
target_link_libraries(google_transform
  Eigen3::Eigen
  glog)

add_subdirectory(benchmark)