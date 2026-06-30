cmake_minimum_required(VERSION 3.10)
project(gazebo_nav_goal_tool)

find_package(ament_cmake REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(rclcpp REQUIRED)
find_package(gazebo REQUIRED)
find_package(Qt5 REQUIRED COMPONENTS Widgets Core)

add_library(gazebo_nav_goal_gui_plugin SHARED
  src/gazebo_nav_goal_gui_plugin.cpp
)

target_include_directories(gazebo_nav_goal_gui_plugin PRIVATE
  ${GAZEBO_INCLUDE_DIRS}
)

target_compile_features(gazebo_nav_goal_gui_plugin PUBLIC cxx_std_17)
target_compile_definitions(gazebo_nav_goal_gui_plugin PRIVATE QT_NO_KEYWORDS)

ament_target_dependencies(gazebo_nav_goal_gui_plugin
  geometry_msgs
  rclcpp
)

target_link_libraries(gazebo_nav_goal_gui_plugin
  ${GAZEBO_LIBRARIES}
  Qt5::Widgets
  Qt5::Core
)

install(
  TARGETS gazebo_nav_goal_gui_plugin
  LIBRARY DESTINATION lib
)

install(
  FILES package.xml
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
