cmake_minimum_required(VERSION 3.8)
project(batch_lio)

if(NOT CMAKE_BUILD_TYPE)
  set(CMAKE_BUILD_TYPE Release)
endif()

set(CMAKE_CXX_STANDARD 14)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

add_compile_options(
  "$<$<COMPILE_LANGUAGE:CXX>:-Wall>"
  "$<$<COMPILE_LANGUAGE:CXX>:-Wextra>"
  "$<$<COMPILE_LANGUAGE:CXX>:-Wno-unused-parameter>"
  "$<$<COMPILE_LANGUAGE:CXX>:-Wno-unused-but-set-variable>")

# ---- optional CUDA representative-map backend (x86-cuda / Jetson-cuda) ----
# CPU-only remains the default so the normal ROS build has no CUDA dependency.
option(BATCH_LIO_ENABLE_CUDA "Build the CUDA representative iVox backend" OFF)
set(BATCH_LIO_CUDA_ARCHITECTURES "" CACHE STRING
    "CUDA architectures (for example 89 for RTX 4070 Ti SUPER, 87 for Jetson Orin)")

# ---- v2: optional aggressive optimizations (Small Point-LIO toolchain; OFF by default) ----
# These can change float results / be non-portable; only enable deliberately and re-verify trajectory.
option(BATCH_LIO_NATIVE_OPT "Enable -march=native (not portable across CPUs)" OFF)
option(BATCH_LIO_FAST_MATH "Enable -ffast-math -fno-math-errno (reassociates float sums; re-verify filter)" OFF)
option(BATCH_LIO_LTO "Enable link-time optimization" OFF)
if(BATCH_LIO_NATIVE_OPT)
    add_compile_options("$<$<COMPILE_LANGUAGE:CXX>:-march=native>")
endif()
if(BATCH_LIO_FAST_MATH)
    add_compile_options(
      "$<$<COMPILE_LANGUAGE:CXX>:-ffast-math>"
      "$<$<COMPILE_LANGUAGE:CXX>:-fno-math-errno>")
endif()
if(BATCH_LIO_LTO)
    add_compile_options("$<$<COMPILE_LANGUAGE:CXX>:-flto>")
    set(CMAKE_INTERPROCEDURAL_OPTIMIZATION TRUE)
endif()

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

if(BATCH_LIO_ENABLE_CUDA)
  include(CheckLanguage)
  check_language(CUDA)
  if(NOT CMAKE_CUDA_COMPILER)
    message(FATAL_ERROR "BATCH_LIO_ENABLE_CUDA=ON but no CUDA compiler was found")
  endif()
  enable_language(CUDA)
  add_library(batch_lio_cuda_backend STATIC
    src/cuda/cuda_representative_ivox.cu)
  set_target_properties(batch_lio_cuda_backend PROPERTIES
    CUDA_STANDARD 14
    CUDA_STANDARD_REQUIRED ON
    POSITION_INDEPENDENT_CODE ON)
  if(BATCH_LIO_CUDA_ARCHITECTURES)
    set_target_properties(batch_lio_cuda_backend PROPERTIES
      CUDA_ARCHITECTURES "${BATCH_LIO_CUDA_ARCHITECTURES}")
  endif()
  target_compile_definitions(batch_lio_cuda_backend PUBLIC BATCH_LIO_ENABLE_CUDA=1)
else()
  add_library(batch_lio_cuda_backend STATIC
    src/cuda/cuda_representative_ivox_stub.cpp)
endif()
target_include_directories(batch_lio_cuda_backend PUBLIC include)

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
  ${PCL_LIBRARIES} Eigen3::Eigen OpenMP::OpenMP_CXX batch_lio_cuda_backend)
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

  ament_add_gtest(test_representative_ivox test/test_representative_ivox.cpp)
  target_include_directories(test_representative_ivox PRIVATE include)

  if(BATCH_LIO_ENABLE_CUDA)
    add_executable(test_cuda_representative_ivox test/test_cuda_representative_ivox.cpp)
    target_link_libraries(test_cuda_representative_ivox batch_lio_cuda_backend)
    add_test(NAME cuda_representative_ivox COMMAND test_cuda_representative_ivox)
  endif()

  find_package(launch_testing_ament_cmake REQUIRED)
  add_launch_test(test/test_smoke.py TIMEOUT 180)
endif()

ament_package()
