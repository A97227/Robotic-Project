cmake_minimum_required(VERSION 3.8)
project(pioneer_interfaces)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rosidl_default_generators REQUIRED)

rosidl_generate_interfaces(${PROJECT_NAME}
    "srv/SetValue.srv"
    "srv/SetTravel.srv"
)

# declare generated messages as dependencies to force generation before compilation
# cf. https://discourse.ros.org/t/ros2-how-to-use-custom-message-in-project-where-its-declared/2071
# rosidl_target_interfaces(${PROJECT_NAME}_debug ${PROJECT_NAME} "rosidl_typesupport_cpp")


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


ament_export_dependencies(rosidl_default_runtime)
ament_package()