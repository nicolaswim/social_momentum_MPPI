cmake_minimum_required(VERSION 3.8)
project(my_social_nav_interfaces)

# Find necessary build tools and message dependencies
find_package(ament_cmake REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)

# This is the CRITICAL missing line to find the generator
find_package(rosidl_default_generators REQUIRED)

# This command tells the system to generate the Python/C++ code
# from your .msg files
rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/HumanArray.msg"
  "msg/HumanPoseVel.msg"
  DEPENDENCIES std_msgs geometry_msgs
)

# Standard testing and package export
if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()