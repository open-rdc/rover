cmake_minimum_required(VERSION 3.5)
project(planner)

# Find dependencies automatically
find_package(ament_cmake_auto REQUIRED)
ament_auto_find_build_dependencies()

# Add C++ executables
ament_auto_add_executable(simple_planner_node src/local_planner/simple_planner_node.cpp)
ament_auto_add_executable(dwa_node src/local_planner/dwa_node.cpp)

# Install Python scripts
install(PROGRAMS
  src/global_planner/global_planner_node.py
  src/nmea_tcp_driver/gga_node.py
  src/nmea_tcp_driver/rmc_node.py
  DESTINATION lib/${PROJECT_NAME}
)

# Install config directory
install(DIRECTORY
  config/
  DESTINATION share/${PROJECT_NAME}/config
  FILES_MATCHING PATTERN "*.csv"
)

# Auto package (includes automatic installation of headers and other resources)
ament_auto_package(USE_SCOPED_HEADER_INSTALL_DIR)
