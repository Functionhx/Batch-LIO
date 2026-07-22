cmake_minimum_required(VERSION 3.8)
project(batch_lio)

if(NOT CMAKE_BUILD_TYPE)
  set(CMAKE_BUILD_TYPE Release)
endif()

set(CMAKE_CXX_STANDARD 14)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

add_compile_options(-Wall -Wextra -Wno-unused-parameter -Wno-unused-but-set-variable)

add_definitions(-DROOT_DIR=\"${CMAKE_CURRENT_SOURCE_DIR}/\")

# ---- OpenMP (preserved from ROS1: auto thread count by core count) ----
message("Current CPU archtecture: ${CMAKE_SYSTEM_PROCESSOR}")
if(CMAKE_SYSTEM_PROCESSOR MATCHES "(x86)|(X86)|(amd64)|(AMD64)")
  include(ProcessorCount)
  ProcessorCount(N)
  message("Processer number: ${N}")
  if(N GREATER 5)
    add_definitions(-DMP_EN)
    add_definitions(-DMP_PROC_NUM=4)
  elseif(N GREATER 3)
    add_definitions(-DMP_EN)
    add_definitions(-DMP_PROC_NUM=4)
  else()
    add_definitions(-DMP_PROC_NUM=1)
  endif()
else()
  add_definitions(-DMP_PROC_NUM=1)
endif()
find_package(OpenMP REQUIRED)

find_package(Eigen3 REQUIRED)
find_package(PCL REQUIRED)
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(visualization_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(pcl_conversions REQUIRED)
find_package(livox_ros_driver2 REQUIRED)

include_directories(
  ${EIGEN3_INCLUDE_DIR}
  ${PCL_INCLUDE_DIRS}
  include
  src
)

link_directories(${PCL_LIBRARY_DIRS})

# ---- the odometry node ----
add_executable(batchlio_mapping
  src/laserMapping.cpp
  src/li_initialization.cpp
  src/parameters.cpp
  src/preprocess.cpp
  src/Estimator.cpp
  src/IMU_Processing.cpp
  src/stage_profiler.cpp)
target_link_libraries(batchlio_mapping
  ${PCL_LIBRARIES} Eigen3::Eigen OpenMP::OpenMP_CXX)
ament_target_dependencies(batchlio_mapping
  rclcpp sensor_msgs nav_msgs geometry_msgs visualization_msgs tf2 tf2_ros pcl_conversions livox_ros_driver2)

install(TARGETS batchlio_mapping
  DESTINATION lib/${PROJECT_NAME})

install(DIRECTORY launch config rviz_cfg
  DESTINATION share/${PROJECT_NAME})

# ---- tests (gtest added here; launch_test added in Plan T13) ----
if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)
  ament_add_gtest(test_deskew test/test_deskew.cpp)
  target_include_directories(test_deskew PRIVATE ${EIGEN3_INCLUDE_DIR} src)
  target_link_libraries(test_deskew Eigen3::Eigen)

  find_package(launch_testing_ament_cmake REQUIRED)
  add_launch_test(test/test_smoke.py TIMEOUT 180)
endif()

ament_package()
