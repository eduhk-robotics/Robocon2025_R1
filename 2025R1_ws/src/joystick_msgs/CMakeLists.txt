cmake_minimum_required(VERSION 3.8)
project(joystick_msgs)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
# Add this line to find the message generation tools
find_package(rosidl_default_generators REQUIRED)

# Add this block to generate messages from .msg files
rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/Joystick.msg"
  # If you had services or actions, you would list them here too
  # "srv/MyService.srv"
  # "action/MyAction.action"
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  set(ament_lint_auto_lint_types "copyright;xmllint;flake8;cpplint;ament_cmake;")
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()