cmake_minimum_required(VERSION 3.8)
project(rm_auto_aim)

# ==============================================================================
# 1. 基础配置：C++标准、编译选项
# ==============================================================================
# 设置C++标准（适配ROS2，建议C++17）
if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# ==============================================================================
# 2. 查找依赖：ROS2组件 + Eigen3 + OpenCV
# ==============================================================================
# ROS2核心依赖
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_components REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(image_transport REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(rm_interfaces REQUIRED)  # RM自定义接口（根据项目实际情况调整）

# OpenCV依赖（解决fillConvexPoly相关编译问题）
find_package(OpenCV 4 REQUIRED
  COMPONENTS
    core
    imgproc
    highgui
    imgcodecs
)

# Eigen3依赖（关键：解决头文件找不到问题）
find_package(Eigen3 3.3 REQUIRED NO_MODULE)

# ==============================================================================
# 3. 头文件路径配置
# ==============================================================================
include_directories(
  # 项目自身头文件
  include
  # Eigen3头文件（手动指定+变量双重保障）
  /usr/include/eigen3
  ${EIGEN3_INCLUDE_DIRS}
  # OpenCV头文件
  ${OpenCV_INCLUDE_DIRS}
  # ROS2依赖头文件
  ${rclcpp_INCLUDE_DIRS}
  ${cv_bridge_INCLUDE_DIRS}
)

# ==============================================================================
# 4. 编译库/可执行文件
# ==============================================================================
# 编译armor_detector核心库（包含pnp_solver和detector）
add_library(armor_detector SHARED
  src/detector/pnp_solver.cpp
  src/detector/detector.cpp
  # 如需添加其他源文件，在此补充
  # src/xxx/xxx.cpp
)

# 链接依赖到armor_detector库
ament_target_dependencies(armor_detector
  rclcpp
  rclcpp_components
  sensor_msgs
  geometry_msgs
  std_msgs
  image_transport
  cv_bridge
  rm_interfaces
)
# 显式链接Eigen3和OpenCV（关键）
target_link_libraries(armor_detector
  Eigen3::Eigen
  ${OpenCV_LIBRARIES}
)

# （可选）如果有可执行节点，添加以下配置
# add_executable(auto_aim_node
#   src/auto_aim_node.cpp
# )
# ament_target_dependencies(auto_aim_node
#   rclcpp
#   armor_detector
# )
# target_link_libraries(auto_aim_node
#   armor_detector
# )

# ==============================================================================
# 5. 安装配置（ROS2必须）
# ==============================================================================
# 安装库文件
install(TARGETS armor_detector
  EXPORT export_${PROJECT_NAME}
  ARCHIVE DESTINATION lib/${PROJECT_NAME}
  LIBRARY DESTINATION lib/${PROJECT_NAME}
  RUNTIME DESTINATION bin/${PROJECT_NAME}
)

# 安装头文件
install(DIRECTORY include/
  DESTINATION include/${PROJECT_NAME}
)

# （可选）安装可执行文件
# install(TARGETS auto_aim_node
#   DESTINATION lib/${PROJECT_NAME}
# )

# ==============================================================================
# 6. 依赖导出 & 包声明
# ==============================================================================
# 导出依赖，让其他包能找到本包
ament_export_include_directories(include)
ament_export_libraries(armor_detector)
ament_export_dependencies(
  rclcpp
  sensor_msgs
  cv_bridge
  Eigen3
  OpenCV
)

# ROS2包声明
ament_package()
