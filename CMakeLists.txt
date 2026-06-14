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

add_library(${PROJECT_NAME}_core
  src/mapping/occupancy_grid.cpp
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
)
  add_executable(${node} src/nodes/${node}.cpp)
  target_link_libraries(${node} ${PROJECT_NAME}_core)
  ament_target_dependencies(${node}
    rclcpp geometry_msgs nav_msgs sensor_msgs
  )
endforeach()

install(TARGETS
  ${PROJECT_NAME}_core
  mapping_node
  planner_node
  controller_node
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
endif()

ament_package()
