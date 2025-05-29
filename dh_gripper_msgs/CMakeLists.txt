cmake_minimum_required(VERSION 3.5)
project(dh_gripper_msgs)

find_package(ament_cmake REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(std_msgs REQUIRED)

rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/GripperCtrl.msg"
  "msg/GripperState.msg"
  "msg/GripperRotCtrl.msg"
  "msg/GripperRotState.msg"
  DEPENDENCIES std_msgs
)

ament_export_dependencies(rosidl_default_runtime)

install(DIRECTORY msg
  DESTINATION share/${PROJECT_NAME}
)

ament_package()


