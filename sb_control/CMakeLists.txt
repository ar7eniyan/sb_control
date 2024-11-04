cmake_minimum_required(VERSION 3.16)
project(ros2_control_demo_example_11 LANGUAGES CXX)

if(CMAKE_CXX_COMPILER_ID MATCHES "(GNU|Clang)")
  add_compile_options(-Wall -Wextra)
endif()

# set the same behavior for windows as it is on linux
set(CMAKE_WINDOWS_EXPORT_ALL_SYMBOLS ON)

# find dependencies
set(THIS_PACKAGE_INCLUDE_DEPENDS
  controller_interface
  hardware_interface
  pluginlib
  rclcpp
  rclcpp_lifecycle
  rcpputils
  realtime_tools
  builtin_interfaces
  std_srvs
  tf2
  tf2_msgs
  ackermann_msgs
  geometry_msgs
  nav_msgs
)

# find dependencies
find_package(backward_ros REQUIRED)
find_package(ament_cmake REQUIRED)
foreach(Dependency IN ITEMS ${THIS_PACKAGE_INCLUDE_DEPENDS})
  find_package(${Dependency} REQUIRED)
endforeach()

## COMPILE
add_library(
  ros2_control_demo_example_11
  SHARED
  hardware/carlikebot_system.cpp
  controller/tricycle_controller.cpp
  controller/odometry.cpp
  controller/traction_limiter.cpp
  controller/steering_limiter.cpp
)
target_compile_features(ros2_control_demo_example_11 PUBLIC cxx_std_17)
target_include_directories(ros2_control_demo_example_11 PUBLIC
$<BUILD_INTERFACE:${PROJECT_SOURCE_DIR}/hardware/include>
$<BUILD_INTERFACE:${PROJECT_SOURCE_DIR}/controller/include>
$<INSTALL_INTERFACE:include/ros2_control_demo_example_11>
)
ament_target_dependencies(
  ros2_control_demo_example_11 PUBLIC
  ${THIS_PACKAGE_INCLUDE_DEPENDS}
)

# Causes the visibility macros to use dllexport rather than dllimport,
# which is appropriate when building the dll but not consuming it.
target_compile_definitions(ros2_control_demo_example_11 PRIVATE "ACKERMANN_STEERING_CONTROLLER_BUILDING_DLL")
# From tricycle controller CMakeLists.txt
target_compile_definitions(ros2_control_demo_example_11 PRIVATE _USE_MATH_DEFINES)

# Export hardware plugins
pluginlib_export_plugin_description_file(hardware_interface ros2_control_demo_example_11.xml)
# Export controllers
pluginlib_export_plugin_description_file(controller_interface ros2_control_demo_example_11.xml)

# INSTALL
install(
  DIRECTORY hardware/include/
  DESTINATION include/ros2_control_demo_example_11
)
install(
  DIRECTORY controller/include/
  DESTINATION include/ros2_control_demo_example_11
)
install(
  DIRECTORY description/launch description/ros2_control description/urdf
  DESTINATION share/ros2_control_demo_example_11
)
install(
  DIRECTORY bringup/launch bringup/config
  DESTINATION share/ros2_control_demo_example_11
)
install(TARGETS ros2_control_demo_example_11
  EXPORT export_ros2_control_demo_example_11
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

if(BUILD_TESTING)
  find_package(ament_cmake_pytest REQUIRED)

  ament_add_pytest_test(example_11_urdf_xacro test/test_urdf_xacro.py)
  ament_add_pytest_test(view_example_11_launch test/test_view_robot_launch.py)
  ament_add_pytest_test(run_example_11_launch test/test_carlikebot_launch.py)
endif()

## EXPORTS
ament_export_targets(export_ros2_control_demo_example_11 HAS_LIBRARY_TARGET)
ament_export_dependencies(${THIS_PACKAGE_INCLUDE_DEPENDS})
ament_package()
