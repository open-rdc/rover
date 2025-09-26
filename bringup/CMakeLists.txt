cmake_minimum_required(VERSION 3.8)
project(bringup)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake_auto REQUIRED)
ament_auto_find_build_dependencies()

ament_auto_add_library(chassis_driver_node src/chassis_driver_node.cpp)

ament_export_include_directories(include)
# ament_export_libraries(chassis_driver)

# executor
add_executable(executor src/execute.cpp)
ament_target_dependencies(executor
    rclcpp
)
target_link_libraries(executor
    chassis_driver_node
)

# install
install(DIRECTORY launch config
    DESTINATION share/${PROJECT_NAME}
)
install(TARGETS
    executor
    DESTINATION lib/${PROJECT_NAME}
)

ament_auto_package(INSTALL_TO_SHARE)
