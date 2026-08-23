cmake_minimum_required(VERSION 3.16)
project(toy_rover)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 20)
endif()

set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(tf2_ros REQUIRED)

add_library(${PROJECT_NAME}_core
  src/mapping/occupancy_grid.cpp
  src/mapping/obstacle_history.cpp
  src/mapping/obstacle_inflation.cpp
  src/control/pose_history.cpp
  src/control/stuck_detector.cpp
  src/planning/astar.cpp
  src/control/pure_pursuit.cpp
  src/control/velocity_limiter.cpp
)

target_include_directories(${PROJECT_NAME}_core PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/src>
  $<INSTALL_INTERFACE:include>
)

target_compile_features(${PROJECT_NAME}_core PUBLIC cxx_std_20)

foreach(node
  mapping_node
  planner_node
  controller_node
  odom_tf_broadcaster_node
)
  add_executable(${node} src/nodes/${node}.cpp)
  target_link_libraries(${node} ${PROJECT_NAME}_core)
  ament_target_dependencies(${node}
    rclcpp geometry_msgs nav_msgs sensor_msgs std_msgs tf2_ros
  )
endforeach()

install(TARGETS
  ${PROJECT_NAME}_core
  mapping_node
  planner_node
  controller_node
  odom_tf_broadcaster_node
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY launch config worlds urdf
  DESTINATION share/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)

  ament_add_gtest(test_astar test/test_astar.cpp)
  target_link_libraries(test_astar ${PROJECT_NAME}_core)
  target_include_directories(test_astar PRIVATE src)

  ament_add_gtest(test_pure_pursuit test/test_pure_pursuit.cpp)
  target_link_libraries(test_pure_pursuit ${PROJECT_NAME}_core)
  target_include_directories(test_pure_pursuit PRIVATE src)

  ament_add_gtest(test_obstacle_history test/test_obstacle_history.cpp)
  target_link_libraries(test_obstacle_history ${PROJECT_NAME}_core)
  target_include_directories(test_obstacle_history PRIVATE src)

  ament_add_gtest(test_obstacle_inflation test/test_obstacle_inflation.cpp)
  target_link_libraries(test_obstacle_inflation ${PROJECT_NAME}_core)
  target_include_directories(test_obstacle_inflation PRIVATE src)

  ament_add_gtest(test_stuck_detector test/test_stuck_detector.cpp)
  target_link_libraries(test_stuck_detector ${PROJECT_NAME}_core)
  target_include_directories(test_stuck_detector PRIVATE src)

  ament_add_gtest(test_pose_history test/test_pose_history.cpp)
  target_link_libraries(test_pose_history ${PROJECT_NAME}_core)
  target_include_directories(test_pose_history PRIVATE src)
endif()

ament_package()
