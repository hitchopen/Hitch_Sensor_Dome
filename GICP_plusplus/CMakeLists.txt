###########################################################
#                                                         #
# Copyright (c)                                           #
#                                                         #
# The Verifiable & Control-Theoretic Robotics (VECTR) Lab #
# University of California, Los Angeles                   #
#                                                         #
# Authors: Kenny J. Chen, Ryan Nemiroff, Brett T. Lopez   #
# Contact: {kennyjchen, ryguyn, btlopez}@ucla.edu         #
#                                                         #
###########################################################

cmake_minimum_required(VERSION 3.16)
project(gicp_localization)
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(visualization_msgs REQUIRED)
find_package(pcl_ros REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_eigen REQUIRED)
find_package(tf2_sensor_msgs REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(PCL REQUIRED)
find_package( Eigen3 REQUIRED )
find_package(Threads REQUIRED)
find_package(LibXml2 REQUIRED)  # URDF parsing for offline aux-LiDAR extrinsics (lidar_concat)
find_package(yaml_cpp_vendor REQUIRED)
find_package(yaml-cpp REQUIRED)
find_package(OpenMP REQUIRED)

if(NOT CMAKE_BUILD_TYPE AND NOT CMAKE_CONFIGURATION_TYPES)
  set(CMAKE_BUILD_TYPE "Release" CACHE STRING "Choose the type of build." FORCE)
  set_property(CACHE CMAKE_BUILD_TYPE PROPERTY STRINGS "Debug" "Release" "MinSizeRel" "RelWithDebInfo")
endif()
set(CMAKE_CXX_STANDARD 17)

set(CMAKE_THREAD_PREFER_PTHREAD TRUE)
set(THREADS_PREFER_PTHREAD_FLAG TRUE)

# Not all machines have <cpuid.h> available
set(CMAKE_TRY_COMPILE_TARGET_TYPE STATIC_LIBRARY)

# rosidl_get_typesupport_target(cpp_typesupport_target
#   ${PROJECT_NAME} rosidl_typesupport_cpp)

include_directories(${PCL_INCLUDE_DIRS})
include_directories(${EIGEN3_INCLUDE_DIR})

include_directories(include)
include_directories(SYSTEM)

# small_gicp is vendored as a header-only dependency. Keeping the exact
# reviewed headers in-tree gives Humble and Jazzy the same registration
# implementation without relying on a distro-specific system package.
add_library(gicp_localization_small_gicp INTERFACE)
target_include_directories(gicp_localization_small_gicp INTERFACE
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/thirdparty/small_gicp/include>
)
target_link_libraries(gicp_localization_small_gicp INTERFACE
  Eigen3::Eigen
  OpenMP::OpenMP_CXX
)

# Localization Node
add_executable(gicp_localization_node src/localization_node.cc src/localization.cc)
target_include_directories(gicp_localization_node PRIVATE ${LIBXML2_INCLUDE_DIR})
target_link_libraries(gicp_localization_node
  ${PCL_LIBRARIES}
  OpenMP::OpenMP_CXX
  Threads::Threads
  gicp_localization_small_gicp
  ${LIBXML2_LIBRARIES}
  yaml-cpp
)
ament_target_dependencies(gicp_localization_node "tf2_ros" "tf2_geometry_msgs" "tf2_eigen" "tf2_sensor_msgs" "pcl_ros" "nav_msgs" "pcl_conversions" "rclcpp" "std_msgs" "sensor_msgs" "geometry_msgs" "visualization_msgs")

# nav_sat_gated_odom — Hitch Sensor Dome RTK-gating republisher.
# Sits between a legacy INS odometry + NavSatFix pair and the localization
# node's gt_odom input, forwarding Odometry only when NavSatFix
# reports RTK-class status with valid cm-grade covariance. Production uses
# the adapter's native solution-type-gated fixed-only stream.
add_executable(nav_sat_gated_odom src/nav_sat_gated_odom.cc)
ament_target_dependencies(nav_sat_gated_odom rclcpp nav_msgs sensor_msgs)

# Binaries
install( TARGETS gicp_localization_node nav_sat_gated_odom
         DESTINATION lib/${PROJECT_NAME}
)
install( PROGRAMS
         scripts/profile_localization_resources.py
         scripts/plot_localization_profile.py
         DESTINATION lib/${PROJECT_NAME}
)
install( DIRECTORY cfg launch
         DESTINATION share/${PROJECT_NAME}
)
# Install the vehicle URDF alongside the package share so the launch's walk-up
# (share/gicp_localization/launch -> share/gicp_localization) resolves the URDF
# in the colcon install tree, not just the source tree. OPTIONAL: the source-tree
# walk-up / static fallback still apply if the file isn't present at install time.
install( FILES ../GLIM_plusplus/config/sensor_dome.urdf
         DESTINATION share/${PROJECT_NAME}
         OPTIONAL
)
file(WRITE ${CMAKE_BINARY_DIR}/test_cpuid.cpp "#include <cpuid.h>")
try_compile(HAS_CPUID ${CMAKE_BINARY_DIR} ${CMAKE_BINARY_DIR}/test_cpuid.cpp)
file(REMOVE ${CMAKE_BINARY_DIR}/test_cpuid.cpp)
if(HAS_CPUID)
  add_compile_definitions(HAS_CPUID)
endif()

# nav_sat_gated_odom doesn't use OpenMP, but compile with the same
# warning hygiene as the rest of the package.
target_compile_options(nav_sat_gated_odom PRIVATE -Wall -Wextra)

if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)

  ament_add_gtest(seyond_timestamp_test
    test/seyond_timestamp_test.cpp
  )
  ament_target_dependencies(seyond_timestamp_test sensor_msgs)
  target_include_directories(seyond_timestamp_test PRIVATE include)

  ament_add_gtest(lidar_fov_gate_test
    test/lidar_fov_gate_test.cpp
  )
  ament_target_dependencies(lidar_fov_gate_test sensor_msgs)
  target_include_directories(lidar_fov_gate_test PRIVATE include)

  ament_add_gtest(map_manifest_test
    test/map_manifest_test.cpp
  )
  ament_target_dependencies(map_manifest_test yaml_cpp_vendor)
  target_link_libraries(map_manifest_test yaml-cpp)
  target_include_directories(map_manifest_test PRIVATE include)

  ament_add_gtest(navsat_fix_gate_test
    test/navsat_fix_gate_test.cpp
  )
  ament_target_dependencies(navsat_fix_gate_test sensor_msgs)
  target_include_directories(navsat_fix_gate_test PRIVATE include)

  ament_add_gtest(small_gicp_backend_test
    test/small_gicp_backend_test.cpp
  )
  target_include_directories(small_gicp_backend_test PRIVATE
    include
    thirdparty/small_gicp/include
  )
  target_link_libraries(small_gicp_backend_test
    ${PCL_LIBRARIES}
    OpenMP::OpenMP_CXX
    gicp_localization_small_gicp
  )
endif()

ament_package()
