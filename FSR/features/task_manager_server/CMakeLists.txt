cmake_minimum_required(VERSION 3.8)
project(task_manager_server)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_action REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(action_msgs REQUIRED)
find_package(rosidl_default_generators REQUIRED)

set(srv_files
  "srv/ChangePlanners.srv"
  # "srv/RunTaskFile.srv"
  "srv/SelectFile.srv"
)

set(msg_files
  "msg/RobotStatus.msg"

)

set(action_files
  "action/RunTaskFile.action"

)
rosidl_generate_interfaces(${PROJECT_NAME}
  ${srv_files}
  ${action_files}
  DEPENDENCIES std_msgs geometry_msgs action_msgs
)

ament_export_dependencies(
  rosidl_default_runtime
  std_msgs
  geometry_msgs
  action_msgs
)

install(DIRECTORY srv/
  DESTINATION share/${PROJECT_NAME}/srv
)

install(DIRECTORY msg/
  DESTINATION share/${PROJECT_NAME}/msg
)

install(DIRECTORY action/
  DESTINATION share/${PROJECT_NAME}/action
)

ament_package()
