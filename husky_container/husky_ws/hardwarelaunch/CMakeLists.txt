cmake_minimum_required(VERSION 3.5)
project(hardwarelaunch)

# Find ament and install helpers
find_package(ament_cmake REQUIRED)

# Install your Python script into <install_prefix>/lib/hardwarelaunch/
install(PROGRAMS
  scripts/run.py
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()
