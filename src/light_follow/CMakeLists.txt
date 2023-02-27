cmake_minimum_required(VERSION 3.8)
project(light_follow)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(cv_bridge REQUIRED)

find_package(asdfr_interfaces REQUIRED)

add_executable(open_light_follow src/open_light_follow.cpp)
add_executable(closed_light_follow src/closed_light_follow.cpp)
add_executable(setpoint_generator src/setpoint_generator.cpp)
add_executable(show_jiwy_pos src/show_jiwy_pos.cpp)

ament_target_dependencies(open_light_follow rclcpp asdfr_interfaces)
ament_target_dependencies(closed_light_follow rclcpp asdfr_interfaces)
ament_target_dependencies(setpoint_generator rclcpp asdfr_interfaces)
ament_target_dependencies(show_jiwy_pos rclcpp asdfr_interfaces sensor_msgs cv_bridge)


install(TARGETS open_light_follow
  DESTINATION lib/${PROJECT_NAME})

install(TARGETS closed_light_follow
  DESTINATION lib/${PROJECT_NAME})

install(TARGETS setpoint_generator
  DESTINATION lib/${PROJECT_NAME})

install(TARGETS show_jiwy_pos
  DESTINATION lib/${PROJECT_NAME})

# Install launch files.
install(DIRECTORY
  launch
  DESTINATION share/${PROJECT_NAME}/
)

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

ament_package()
