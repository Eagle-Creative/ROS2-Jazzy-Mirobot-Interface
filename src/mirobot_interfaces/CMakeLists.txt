cmake_minimum_required(VERSION 3.8)
project(mirobot_interfaces)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(serial_driver REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(mirobot_msgs REQUIRED)


add_executable(mirobot_gcode_writer src/mirobot_gcode_writer.cpp)
ament_target_dependencies(mirobot_gcode_writer rclcpp sensor_msgs std_msgs serial_driver mirobot_msgs)

install(
  TARGETS
    mirobot_gcode_writer
  DESTINATION lib/${PROJECT_NAME}
)

ament_export_dependencies(rosidl_default_runtime)
if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

# Install Python scripts
ament_python_install_package(scripts)

install(
  PROGRAMS
  scripts/mirobot_interface.py
  scripts/mirobot_std_functions.py
  DESTINATION lib/${PROJECT_NAME}
)

# Install launch files
install(
  DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}/
)

ament_package()
