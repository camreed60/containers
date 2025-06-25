cmake_minimum_required(VERSION 3.5)
project(smartagurdf)

find_package(ament_cmake REQUIRED)

# Install config files (e.g. .yaml)
install(
  DIRECTORY config
  DESTINATION share/${PROJECT_NAME}/config
  FILES_MATCHING PATTERN "*.yaml"
)

# Install launch scripts (.py)
install(
  DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}/launch
  FILES_MATCHING PATTERN "*.py"
)

# Install meshes (all files)
install(
  DIRECTORY meshes
  DESTINATION share/${PROJECT_NAME}/meshes
)

# Install URDF/XACROs
install(
  DIRECTORY urdf
  DESTINATION share/${PROJECT_NAME}/urdf
  FILES_MATCHING PATTERN "*.xacro"
)

ament_package()
