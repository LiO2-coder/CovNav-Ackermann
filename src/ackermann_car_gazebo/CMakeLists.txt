cmake_minimum_required(VERSION 3.0.2)
project(ackermann_car_gazebo)

find_package(catkin REQUIRED COMPONENTS
  gazebo_msgs
  geometry_msgs
  message_filters
  message_generation
  nav_msgs
  roscpp
  rospy
  sensor_msgs
  std_msgs
  urdf
)

find_package(cmake_modules REQUIRED)
find_package(Eigen REQUIRED)
find_package(osqp_vendor REQUIRED)
find_path(OSQP_INCLUDE_DIR
  NAMES osqp/osqp.h
  HINTS ${osqp_vendor_INCLUDE_DIRS}
)
find_library(OSQP_LIBRARY
  NAMES osqp
  HINTS ${osqp_vendor_LIBRARY_DIRS}
)
if(NOT OSQP_INCLUDE_DIR OR NOT OSQP_LIBRARY)
  message(FATAL_ERROR "OSQP was not found. Install ros-noetic-osqp-vendor and ros-noetic-osqp.")
endif()

add_message_files(
  FILES
  RtkImuEkfState.msg
)

add_service_files(
  FILES
  PlanCoveragePath.srv
)

generate_messages(
  DEPENDENCIES
  geometry_msgs
  nav_msgs
  std_msgs
)

catkin_package(
  CATKIN_DEPENDS
    gazebo_msgs
    geometry_msgs
    message_filters
    message_runtime
    nav_msgs
    roscpp
    rospy
    sensor_msgs
    std_msgs
    urdf
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${Eigen_INCLUDE_DIRS}
  ${osqp_vendor_INCLUDE_DIRS}
  ${OSQP_INCLUDE_DIR}
)

add_library(coverage_path_planner
  src/coverage_path_planner.cpp
)
add_dependencies(coverage_path_planner
  ${${PROJECT_NAME}_EXPORTED_TARGETS}
  ${catkin_EXPORTED_TARGETS}
)
target_link_libraries(coverage_path_planner
  ${catkin_LIBRARIES}
)

add_executable(dual_rtk_imu_ekf_node
  src/dual_rtk_imu_ekf_node.cpp
)
add_dependencies(dual_rtk_imu_ekf_node
  ${${PROJECT_NAME}_EXPORTED_TARGETS}
  ${catkin_EXPORTED_TARGETS}
)
target_link_libraries(dual_rtk_imu_ekf_node
  ${catkin_LIBRARIES}
)

add_executable(coverage_path_planner_node
  src/coverage_path_planner_node.cpp
)
add_dependencies(coverage_path_planner_node
  ${${PROJECT_NAME}_EXPORTED_TARGETS}
  ${catkin_EXPORTED_TARGETS}
)
target_link_libraries(coverage_path_planner_node
  coverage_path_planner
  ${catkin_LIBRARIES}
)

add_executable(mpc_controller_node
  src/mpc_controller_node.cpp
)
add_dependencies(mpc_controller_node
  ${${PROJECT_NAME}_EXPORTED_TARGETS}
  ${catkin_EXPORTED_TARGETS}
)
target_link_libraries(mpc_controller_node
  ${catkin_LIBRARIES}
  ${osqp_vendor_LIBRARIES}
  ${OSQP_LIBRARY}
)

if(CATKIN_ENABLE_TESTING)
  catkin_add_gtest(test_coverage_path_planner
    test/test_coverage_path_planner.cpp
  )
  if(TARGET test_coverage_path_planner)
    target_link_libraries(test_coverage_path_planner
      coverage_path_planner
      ${catkin_LIBRARIES}
    )
  endif()
endif()

install(TARGETS
    coverage_path_planner
    coverage_path_planner_node
    dual_rtk_imu_ekf_node
    mpc_controller_node
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(DIRECTORY include/${PROJECT_NAME}/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
)

catkin_install_python(PROGRAMS
  scripts/coverage_navigation_gui.py
  scripts/cmd_vel_to_tricycle.py
  scripts/dual_rtk_publisher.py
  scripts/sonar_range_marker.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(DIRECTORY config launch rviz urdf worlds
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)
