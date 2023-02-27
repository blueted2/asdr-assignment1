cmake_minimum_required(VERSION 3.8)
project(light_position)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(asdfr_interfaces REQUIRED)

# uncomment the following section in order to fill in
# further dependencies manually.
# find_package(<dependency> REQUIRED)

add_executable(brightness src/brightness.cpp)
add_executable(light_position src/light_position.cpp)
add_executable(show_light_position src/show_light_position.cpp)
add_executable(fake_light src/fake_light.cpp)

ament_target_dependencies(brightness rclcpp std_msgs sensor_msgs cv_bridge)
ament_target_dependencies(light_position rclcpp std_msgs sensor_msgs cv_bridge asdfr_interfaces)
ament_target_dependencies(show_light_position rclcpp std_msgs sensor_msgs cv_bridge asdfr_interfaces)
ament_target_dependencies(fake_light rclcpp asdfr_interfaces cv_bridge)

install(TARGETS brightness
  DESTINATION lib/${PROJECT_NAME})
  
install(TARGETS light_position
  DESTINATION lib/${PROJECT_NAME})

install(TARGETS show_light_position
  DESTINATION lib/${PROJECT_NAME})

install(TARGETS fake_light
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
