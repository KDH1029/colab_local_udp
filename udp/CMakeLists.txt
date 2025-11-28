cmake_minimum_required(VERSION 3.8)
project(udp)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(humanoid_interfaces REQUIRED)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # comment the line when a copyright and license is added to all source files
  set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # comment the line when this package is in a git repo and when
  # a copyright and license is added to all source files
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

add_executable(udp_sender src/udp_sender.cpp)
ament_target_dependencies(udp_sender rclcpp humanoid_interfaces)
add_executable(udp_receiver src/udp_receiver.cpp)
ament_target_dependencies(udp_receiver rclcpp humanoid_interfaces)

install(TARGETS
  udp_sender
  udp_receiver
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()
