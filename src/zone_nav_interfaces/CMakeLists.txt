cmake_minimum_required(VERSION 3.8)
project(zone_nav_interfaces)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# Find dependencies
find_package(ament_cmake REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(geometry_msgs REQUIRED)

# Generate interfaces
rosidl_generate_interfaces(${PROJECT_NAME}
  "action/GoToZone.action"
  "action/FollowPath.action"
  "action/SwitchStackMode.action"
  "srv/SetStackMode.srv"
  "srv/SaveZone.srv"
  "srv/GoToZone.srv"
  "srv/UploadMap.srv"
  "srv/SetActiveMap.srv"
  "srv/GetActiveMap.srv"
  "srv/DeleteZone.srv"
  "srv/UpdateZoneParams.srv"
  "srv/ReorderZones.srv"
  "srv/SavePath.srv"
  "srv/DeletePath.srv"
  "srv/FollowPath.srv"
  "srv/SaveLayout.srv"
  "srv/LoadLayout.srv"
  "srv/DeleteLayout.srv"
  "srv/SetMaxSpeed.srv"
  "srv/SetControlMode.srv"
  "srv/SetSafetyOverride.srv"
  "srv/SetEStop.srv"
  DEPENDENCIES geometry_msgs
)

ament_export_dependencies(rosidl_default_runtime)
ament_package()
