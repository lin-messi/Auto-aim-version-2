cmake_minimum_required(VERSION 3.8)
project(rm_hardware_driver)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

set(CMAKE_CXX_STANDARD 17)

# ==================== 依赖 ====================
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_components REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(image_transport REQUIRED)
find_package(OpenCV REQUIRED)
find_package(rm_interfaces REQUIRED)

# ==================== 串口驱动节点 ====================
add_library(serial_driver_node SHARED
  src/serial_driver_node.cpp
)
target_include_directories(serial_driver_node PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)
target_link_libraries(serial_driver_node
  pthread
)
ament_target_dependencies(serial_driver_node
  rclcpp
  rclcpp_components
  rm_interfaces
)
rclcpp_components_register_nodes(serial_driver_node
  "rm_auto_aim::SerialDriverNode"
)

# ==================== 相机驱动节点 ====================
add_library(camera_driver_node SHARED
  src/camera_driver_node.cpp
)
target_include_directories(camera_driver_node PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)
target_link_libraries(camera_driver_node
  ${OpenCV_LIBS}
)
ament_target_dependencies(camera_driver_node
  rclcpp
  rclcpp_components
  sensor_msgs
  cv_bridge
  image_transport
)
rclcpp_components_register_nodes(camera_driver_node
  "rm_auto_aim::CameraDriverNode"
)

# ==================== 安装 ====================
install(DIRECTORY include/
  DESTINATION include
)

install(TARGETS
  serial_driver_node
  camera_driver_node
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

ament_package()
