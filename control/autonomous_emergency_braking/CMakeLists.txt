cmake_minimum_required(VERSION 3.14)
project(autonomous_emergency_braking)

find_package(autoware_cmake REQUIRED)
autoware_package()

find_package(PCL REQUIRED)

include_directories(
  include
  SYSTEM
  ${PCL_INCLUDE_DIRS}
)

set(AEB_NODE ${PROJECT_NAME}_node)
ament_auto_add_library(${AEB_NODE} SHARED
  src/node.cpp
)

rclcpp_components_register_node(${AEB_NODE}
  PLUGIN "autoware::motion::control::autonomous_emergency_braking::AEB"
  EXECUTABLE ${PROJECT_NAME}
)


if(BUILD_TESTING)
  ament_add_gtest(test_${PROJECT_NAME}
    test/aeb_node_test.cpp
  )

  target_link_libraries(test_${PROJECT_NAME}
    ${PROJECT_NAME}
  )

  target_include_directories(test_${PROJECT_NAME} PRIVATE src)
endif()


ament_auto_package(
  INSTALL_TO_SHARE
  launch
  config
)
