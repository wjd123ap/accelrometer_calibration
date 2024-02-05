cmake_minimum_required(VERSION 3.0.2)
project(imu_calibration)
find_package(Eigen3 REQUIRED)
add_library(imu_i2c src/IMU_i2c.c)
add_library(imu_lib src/imu_lib.cpp)
target_link_libraries(imu_lib imu_i2c)
include_directories(
  include
  ${EIGEN3_INCLUDE_DIRS}
  )
add_executable(main src/main.cpp)
target_link_libraries(main imu_lib)