cmake_minimum_required(VERSION 3.8)
project(hexabot)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
# uncomment the following section in order to fill in
# further dependencies manually.
# find_package(<dependency> REQUIRED)
# find_package(gazebo REQUIRED)

# set(dependencies
#   "geometry_msgs"
#   "rclcpp"
#   "sensor_msgs"
#   "tf2"
# )

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # comment the line when a copyright and license is added to all source files
  # set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # comment the line when this package is in a git repo and when
  # a copyright and license is added to all source files
  # set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

install(
  DIRECTORY 
    launch
    urdf
    meshes
    models
    config
    scripts
  DESTINATION share/${PROJECT_NAME}
)

install(
  PROGRAMS
    launch/gazebo_control_rsp.launch.py
    launch/gazebo_no_control_rsp.launch.py
    launch/robot_state_publisher.launch.py
    launch/single_leg_control.launch.py
    launch/initialize_legs.launch.py
    launch/tripod_gait.launch.py
    scripts/single_leg_control.py
    scripts/multi_leg_control.py
    scripts/tripod_gait_control.py
  DESTINATION lib/${PROJECT_NAME}
)

# ament_export_include_directories(include)
# ament_export_dependencies(gazebo_ros_pkgs)
# ament_export_dependencies(geometry_msgs)
# ament_export_dependencies(rclcpp)
# ament_export_dependencies(sensor_msgs)
# ament_export_dependencies(tf2)

ament_package()
