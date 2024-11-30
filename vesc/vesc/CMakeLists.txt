cmake_minimum_required(VERSION 3.5)
project(vesc)

find_package(ament_cmake_auto REQUIRED)
ament_auto_find_build_dependencies()

# Launch 파일 설치
install(
  DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}
)

# Params 파일 설치
# install(
#   DIRECTORY params
#   DESTINATION share/${PROJECT_NAME}
# )

ament_auto_package()
