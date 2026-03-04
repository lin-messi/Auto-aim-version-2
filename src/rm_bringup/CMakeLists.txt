cmake_minimum_required(VERSION 3.8)
project(rm_bringup)

find_package(ament_cmake REQUIRED)

# 安装launch文件
install(DIRECTORY launch/
  DESTINATION share/${PROJECT_NAME}/launch
)

# 安装配置文件
install(DIRECTORY config/
  DESTINATION share/${PROJECT_NAME}/config
)

ament_package()
