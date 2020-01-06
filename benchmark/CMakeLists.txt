
find_package(benchmark REQUIRED)
find_package(GTSAM REQUIRED)
find_package(mrpt-poses REQUIRED)
find_package(wave_geometry REQUIRED)

# set(CMAKE_CXX_STANDARD 17)

set(CMAKE_CXX_FLAG ${GTSAM_CMAKE_CXX_FLAG})
message("************gtsam cmake cxx flag GTSAM_CMAKE_CXX_FLAG")

add_executable(geometry_benchmark
  geometry_benchmark.cpp)
target_link_libraries(geometry_benchmark
  google_transform
  benchmark::benchmark
  gtsam
  mrpt::poses
  wave_geometry)