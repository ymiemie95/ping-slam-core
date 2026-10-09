cmake_minimum_required(VERSION 3.10)
project(sonar_slam_frontend)

set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_FLAGS_RELEASE "-O3")
if(NOT CMAKE_CONFIGURATION_TYPES AND NOT CMAKE_BUILD_TYPE)
  set(CMAKE_BUILD_TYPE Release)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(std_srvs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(eigen3_cmake_module REQUIRED)
find_package(Eigen3 3.3 REQUIRED)
find_package(ping360_sonar_msgs REQUIRED)
find_package(waterlinked_a50 REQUIRED)
find_package(px4_msgs REQUIRED)
find_package(mocap4r2_msgs REQUIRED)

################################################################################
# generalhelpfullib — small Eigen/tf2 helpers (from underwaterSLAM_2D_lowCost,
# BSD; see NOTICE). Used by the front-end nodes and exported as a library.
################################################################################
add_library(generalhelpfullib SHARED
  src/generalHelpfulTools/src/generalHelpfulTools.cpp)
ament_target_dependencies(generalhelpfullib tf2 Eigen3)
target_include_directories(generalhelpfullib PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/src/generalHelpfulTools/include>
  $<INSTALL_INTERFACE:include>)

################################################################################
# ekfNode — loosely coupled DVL/IMU EKF odometry
################################################################################
add_executable(ekfNode
  src/stateEstimation/ekfDVLnode.cpp
  src/stateEstimation/ekfDVL.cpp)
ament_target_dependencies(ekfNode
  rclcpp std_msgs sensor_msgs geometry_msgs waterlinked_a50 px4_msgs tf2 Eigen3)
target_link_libraries(ekfNode generalhelpfullib)

################################################################################
# sonarToLaserscan — Ping360 echoes -> motion-compensated, densified /scan
################################################################################
add_executable(sonarToLaserscan src/sonarToLaserscan/sonarToLaserscan.cpp)
ament_target_dependencies(sonarToLaserscan
  rclcpp sensor_msgs geometry_msgs nav_msgs ping360_sonar_msgs
  tf2 tf2_ros tf2_geometry_msgs Eigen3)
target_link_libraries(sonarToLaserscan generalhelpfullib)

################################################################################
# rosGtSaver — mocap ground truth -> TUM trajectory file
################################################################################
add_executable(rosGtSaver src/tools/rosGtSaver.cpp)
ament_target_dependencies(rosGtSaver rclcpp mocap4r2_msgs std_srvs Eigen3)
target_link_libraries(rosGtSaver generalhelpfullib)

################################################################################
# install / export
################################################################################
install(DIRECTORY src/generalHelpfulTools/include/ DESTINATION include)
install(TARGETS generalhelpfullib
  EXPORT export_${PROJECT_NAME}
  LIBRARY DESTINATION lib
  ARCHIVE DESTINATION lib
  RUNTIME DESTINATION bin
  INCLUDES DESTINATION include)
install(TARGETS ekfNode sonarToLaserscan rosGtSaver
  DESTINATION lib/${PROJECT_NAME})
install(PROGRAMS
  scripts/analyze_scan_bag.py
  ${CMAKE_CURRENT_SOURCE_DIR}/../../scripts/traj_recorder.py
  ${CMAKE_CURRENT_SOURCE_DIR}/../../scripts/traj_visualizer.py
  ${CMAKE_CURRENT_SOURCE_DIR}/../../scripts/map_recolor.py
  DESTINATION lib/${PROJECT_NAME})

# Keep the workspace-level runtime configuration available when a launch file
# is invoked from outside the repository root.
install(FILES
  ${CMAKE_CURRENT_SOURCE_DIR}/../../config/slamSettings.yaml
  ${CMAKE_CURRENT_SOURCE_DIR}/../../config/sonar_to_laserscan_params.yaml
  ${CMAKE_CURRENT_SOURCE_DIR}/../../config/sonar_slam_backend_params.yaml
  ${CMAKE_CURRENT_SOURCE_DIR}/../../config/paperView.rviz
  ${CMAKE_CURRENT_SOURCE_DIR}/../../config/slamView.rviz
  DESTINATION share/${PROJECT_NAME}/config)

ament_export_include_directories(include)
ament_export_targets(export_${PROJECT_NAME} HAS_LIBRARY_TARGET)
ament_export_dependencies(eigen3_cmake_module Eigen3 tf2)
ament_package()
