cmake_minimum_required(VERSION 3.14)
project(tier4_rosbag_tool_rviz_plugin)

find_package(autoware_cmake REQUIRED)
autoware_package()
find_package(OpenCV REQUIRED)
find_package(Qt5 REQUIRED COMPONENTS Core Widgets)
find_package(rosbag2_transport REQUIRED)

set(QT_LIBRARIES Qt5::Widgets)
set(CMAKE_AUTOMOC ON)
set(CMAKE_INCLUDE_CURRENT_DIR ON)

include_directories(
  ${OpenCV_INCLUDE_DIRS}
  ${rosbag2_transport_INCLUDE_DIRS}
)

ament_auto_add_library(${PROJECT_NAME} SHARED
  src/rosbag_tool_panel.hpp
  src/rosbag_tool_panel.cpp
)

target_link_libraries(${PROJECT_NAME}
  ${QT_LIBRARIES}
  ${OpenCV_LIBRARIES}
  ${rosbag2_transport_LIBRARIES}
)

pluginlib_export_plugin_description_file(rviz_common plugins/plugin_description.xml)

ament_auto_package(
  INSTALL_TO_SHARE
  plugins
)
