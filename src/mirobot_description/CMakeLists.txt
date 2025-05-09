cmake_minimum_required(VERSION 3.8)
project(mirobot_description)

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

# uncomment the following section in order to fill in
# further dependencies manually.
# find_package(<dependency> REQUIRED)

ament_export_dependencies(rosidl_default_runtime)
ament_export_dependencies(mirobot_interfaces)
if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # comment the line when a copyright and license is added to all source files
  set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # comment the line when this package is in a git repo and when
  # a copyright and license is added to all source files
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

add_executable(mirobot_gcode_writer src/mirobot_gcode_writer.cpp)
ament_target_dependencies(mirobot_gcode_writer rclcpp sensor_msgs std_msgs serial_driver)

add_executable(mirobot_gcode_writer_isaac src/mirobot_gcode_writer_isaac.cpp)
ament_target_dependencies(mirobot_gcode_writer_isaac rclcpp sensor_msgs std_msgs serial_driver)

ament_python_install_package(scripts)
install(
  TARGETS
    mirobot_gcode_writer
    mirobot_gcode_writer_isaac
  DESTINATION 
    lib/${PROJECT_NAME}
)



install(
  DIRECTORY
    launch
    meshes
    rviz
    urdf
  DESTINATION
    share/${PROJECT_NAME}/
)

ament_package()
