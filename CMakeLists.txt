cmake_minimum_required(VERSION 3.8)
project(turtle_visualizer)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(turtlesim REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(OpenCV REQUIRED)

include_directories(
  include
  ${OpenCV_INCLUDE_DIRS}
)

# turtle_visualizer_node executable
add_executable(turtle_visualizer_node src/turtle_visualizer_node.cpp)
ament_target_dependencies(turtle_visualizer_node 
  rclcpp 
  turtlesim 
  geometry_msgs
  cv_bridge
  nav_msgs
)
target_link_libraries(turtle_visualizer_node
  ${OpenCV_LIBRARIES}
)

install(TARGETS
  turtle_visualizer_node
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY
  launch
  DESTINATION share/${PROJECT_NAME}/
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
