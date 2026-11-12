cmake_minimum_required(VERSION 3.8)
project(nontok_interfaces)

find_package(ament_cmake REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(builtin_interfaces REQUIRED)
find_package(std_msgs REQUIRED)

rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/BaseStatus.msg"
  "msg/TagDetection.msg"
  "msg/TagDetectionArray.msg"
  "msg/TrayState.msg"
  "msg/MissionState.msg"
  "msg/LocalizationStatus.msg"
  "srv/SetTray.srv"
  "srv/ClassifyItem.srv"
  "action/DockToTags.action"
  DEPENDENCIES builtin_interfaces std_msgs
)

ament_export_dependencies(rosidl_default_runtime)
ament_package()
