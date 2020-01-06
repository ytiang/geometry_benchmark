
find_package(benchmark REQUIRED)
find_package(GTSAM REQUIRED)
find_package(mrpt-poses REQUIRED)
find_package(wave_geometry REQUIRED)

include_directories(
  ${PYTHON_INCLUDE_DIRS}
)

add_executable(geometry_benchmark
  geometry_benchmark.cpp)
target_link_libraries(geometry_benchmark
  google_transform
  benchmark::benchmark
  gtsam
  mrpt::poses
  wave_geometry)