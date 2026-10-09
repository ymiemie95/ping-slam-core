cmake_minimum_required(VERSION 3.5)
project(sonar_slam_backend)

set(CMAKE_BUILD_TYPE Release) #None, Debug, Release, RelWithDebInfo, MinSizeRel
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()
cmake_policy(SET CMP0077 NEW)

set(CMAKE_MODULE_PATH ${CMAKE_MODULE_PATH} "${CMAKE_CURRENT_LIST_DIR}/CMake/")
list(APPEND CMAKE_MODULE_PATH ${CMAKE_CURRENT_LIST_DIR}/lib/karto_sdk/cmake)

find_package(ament_cmake REQUIRED)
find_package(message_filters REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(visualization_msgs REQUIRED)
find_package(pluginlib REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(tf2_sensor_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(std_srvs REQUIRED)
find_package(builtin_interfaces REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(interactive_markers REQUIRED)

# Karto SDK (its separate license and upstream notices are retained in lib/karto_sdk).
set(BUILD_SHARED_LIBS ON)
add_subdirectory(lib/karto_sdk)

set(CMAKE_INCLUDE_CURRENT_DIR ON)

set(dependencies
  rclcpp
  message_filters
  nav_msgs
  sensor_msgs
  tf2
  tf2_ros
  visualization_msgs
  pluginlib
  tf2_geometry_msgs
  tf2_sensor_msgs
  std_msgs
  std_srvs
  builtin_interfaces
  interactive_markers
)

set(libraries
    toolbox_common
    ceres_solver_plugin
    sync_sonar_slam_backend
)

find_package(PkgConfig REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(CSparse REQUIRED)
find_package(Cholmod REQUIRED)
find_package(LAPACK REQUIRED)
find_package(Ceres REQUIRED COMPONENTS SuiteSparse)

set(CMAKE_INCLUDE_CURRENT_DIR ON)
find_package(Boost REQUIRED system serialization filesystem thread)

include_directories(include ${EIGEN3_INCLUDE_DIRS}
                            ${CHOLMOD_INCLUDE_DIR}
                            ${Boost_INCLUDE_DIRS}
                            ${TBB_INCLUDE_DIRS}
                            ${CERES_INCLUDES}
)

add_definitions(${EIGEN3_DEFINITIONS})

rosidl_generate_interfaces(${PROJECT_NAME}
  srvs/Pause.srv
  srvs/ClearQueue.srv
  srvs/ToggleInteractive.srv
  srvs/Clear.srv
  srvs/SaveMap.srv
  srvs/LoopClosure.srv
  srvs/DeserializePoseGraph.srv
  srvs/SerializePoseGraph.srv
  DEPENDENCIES builtin_interfaces geometry_msgs std_msgs nav_msgs visualization_msgs
)

#### Ceres Plugin
add_library(ceres_solver_plugin solvers/ceres_solver.cpp)
ament_target_dependencies(ceres_solver_plugin ${dependencies})
target_link_libraries(ceres_solver_plugin ${CERES_LIBRARIES}
                                          ${Boost_LIBRARIES}
                                          ${TBB_LIBRARIES}
                                          kartoSlamToolbox
)
rosidl_target_interfaces(ceres_solver_plugin ${PROJECT_NAME} "rosidl_typesupport_cpp")
pluginlib_export_plugin_description_file(sonar_slam_backend solver_plugins.xml)

#### Tool lib for mapping
add_library(toolbox_common src/sonar_slam_backend_common.cpp src/map_saver.cpp src/loop_closure_assistant.cpp src/laser_utils.cpp src/slam_mapper.cpp)
ament_target_dependencies(toolbox_common
  ${dependencies}
)
target_link_libraries(toolbox_common kartoSlamToolbox ${Boost_LIBRARIES})
rosidl_target_interfaces(toolbox_common ${PROJECT_NAME} "rosidl_typesupport_cpp")

#### Mapping executibles
add_library(sync_sonar_slam_backend src/sonar_slam_backend_sync.cpp)
target_link_libraries(sync_sonar_slam_backend toolbox_common kartoSlamToolbox ${Boost_LIBRARIES})
add_executable(sync_sonar_slam_backend_node src/sonar_slam_backend_sync_node.cpp)
target_link_libraries(sync_sonar_slam_backend_node sync_sonar_slam_backend)

#### Install
install(TARGETS sync_sonar_slam_backend_node
                ${libraries}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY include/
  DESTINATION include
)

install(FILES solver_plugins.xml
  DESTINATION share
)

ament_export_include_directories(include)
ament_export_libraries(${libraries})
ament_export_dependencies(${dependencies})
ament_package()
