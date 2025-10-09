cmake_minimum_required(VERSION 3.5)
project(planner)

# Find dependencies automatically
find_package(ament_cmake_auto REQUIRED)
ament_auto_find_build_dependencies()

# Add C++ executables
ament_auto_add_executable(local_planner_node src/local_planner/local_planner_node.cpp)

# Install Python scripts
install(PROGRAMS
  src/global_planner/global_planner_node.py
  DESTINATION lib/${PROJECT_NAME}
)

# Auto package (includes automatic installation of headers and other resources)
ament_auto_package()
