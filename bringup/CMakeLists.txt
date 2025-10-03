cmake_minimum_required(VERSION 3.8)
project(bringup)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake_auto REQUIRED)
ament_auto_find_build_dependencies()

ament_export_include_directories(include)

# chassis_driver_node
ament_auto_add_executable(chassis_driver src/chassis_driver_node.cpp)

# install
install(DIRECTORY launch config
    DESTINATION share/${PROJECT_NAME}
)
install(TARGETS
    chassis_driver
    DESTINATION lib/${PROJECT_NAME}
)

ament_auto_package(INSTALL_TO_SHARE)
