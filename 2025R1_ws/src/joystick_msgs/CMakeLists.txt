cmake_minimum_required(VERSION 3.8)
project(joystick_msgs)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
# Important: Add this dependency for message generation
find_package(rosidl_default_generators REQUIRED)

# This is the crucial part for generating messages
rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/Joystick.msg"
  # If you had services, you would list them here:
  # "srv/MyService.srv"
  # If you had actions, you would list them here:
  # "action/MyAction.action"
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  set(ament_lint_auto_lint_types test)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()

