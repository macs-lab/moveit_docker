cmake_minimum_required(VERSION 3.8)
project(inspection_robot_service)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(moveit_ros_planning_interface REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclpy REQUIRED)
find_package(std_msgs REQUIRED) # ADD PACKAGE
find_package(ament_cmake_python REQUIRED)
find_package(inspection_srvs REQUIRED)  # ADD PACKAGE

add_executable(inspection_robot_service src/inspection_robot_service.cpp) # Name in launch file, then executable in launch file

target_include_directories(inspection_robot_service PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
  ${moveit_msgs_INCLUDE_DIRS})
target_compile_features(inspection_robot_service PUBLIC c_std_99 cxx_std_17)  # Require C99 and C++17
ament_target_dependencies(inspection_robot_service
  # inspection_msgs
  inspection_srvs
  "moveit_ros_planning_interface"
  "rclcpp"
  "rclpy"
)

install(TARGETS 
  inspection_robot_service
  DESTINATION lib/${PROJECT_NAME})

# Install launch and config files.
install(DIRECTORY
  launch config
  DESTINATION share/${PROJECT_NAME}/
)

## Python stuff. Not in a standard c++ only package
# Install Python modules
ament_python_install_package(${PROJECT_NAME})

# Install Python executables
install(PROGRAMS
  src/python_pose_client.py
  src/send_pose.py
  src/focus_manual_move.py
  src/inverse_python_pose_client.py
  src/single_speed_autofocus.py
  src/multi_speed_autofocus.py

  DESTINATION lib/${PROJECT_NAME})
  
ament_package()
