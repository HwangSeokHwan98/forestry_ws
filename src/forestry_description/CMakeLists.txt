cmake_minimum_required(VERSION 3.5)
project(forestry_description)

# ROS 2 패키지 설정
find_package(ament_cmake REQUIRED)

# 설치할 디렉토리 지정
install(DIRECTORY config launch meshes urdf
  DESTINATION share/${PROJECT_NAME}/launch
)

ament_package()
