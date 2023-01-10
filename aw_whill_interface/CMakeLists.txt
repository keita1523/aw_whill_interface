cmake_minimum_required(VERSION 3.14)
project(aw_whill_interface)

find_package(autoware_cmake REQUIRED)
autoware_package()

ament_auto_add_library(aw_whill_interface SHARED
  src/aw_whill_interface_node.cpp
)

target_link_libraries(aw_whill_interface)

rclcpp_components_register_node(aw_whill_interface
  PLUGIN "sample_module::AwWhillInterface"
  EXECUTABLE aw_whill_interface_node
)

ament_auto_package(
  INSTALL_TO_SHARE
  # launch
  # config
)