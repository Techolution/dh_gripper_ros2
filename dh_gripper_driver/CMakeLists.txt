cmake_minimum_required(VERSION 3.5)
project(dh_gripper_driver)

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(dh_gripper_msgs REQUIRED)

# Export include directories for downstream packages
ament_export_include_directories(
  include
  include/dh_gripper_driver/include
)

add_executable(dh_gripper_driver
  include/dh_gripper_driver/src/dh_device.cpp
  include/dh_gripper_driver/src/dh_ag95_can.cpp 
  include/dh_gripper_driver/src/dh_dh3_can.cpp 
  include/dh_gripper_driver/src/dh_lagacy_gripper.cpp 
  include/dh_gripper_driver/src/dh_modbus_gripper.cpp 
  include/dh_gripper_driver/src/dh_rgi.cpp 
  src/dh_gripper_driver.cpp
)
target_include_directories(dh_gripper_driver PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include/dh_gripper_driver/include>
  $<INSTALL_INTERFACE:include>
)

add_executable(dh_gripper_driver_test
  src/dh_gripper_Test.cpp
)
target_include_directories(dh_gripper_driver_test PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include/dh_gripper_driver/include>
  $<INSTALL_INTERFACE:include>
)

ament_target_dependencies(dh_gripper_driver rclcpp std_msgs sensor_msgs dh_gripper_msgs)
ament_target_dependencies(dh_gripper_driver_test rclcpp std_msgs sensor_msgs dh_gripper_msgs)

install(TARGETS
  dh_gripper_driver
  dh_gripper_driver_test
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY include launch
  DESTINATION share/${PROJECT_NAME}
)
ament_package()