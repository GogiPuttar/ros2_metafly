cmake_minimum_required(VERSION 3.8)
project(metafly_control)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
# uncomment the following section in order to fill in
# further dependencies manually.
# find_package(<dependency> REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(std_srvs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(metafly_interfaces REQUIRED)
find_package(rosidl_default_generators REQUIRED)

# Add the path to libserial library
find_library(LIBSERIAL libserial.so)

include_directories(/usr/include/libserial/)

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

add_executable(controller src/controller.cpp)
ament_target_dependencies(controller rclcpp tf2_ros tf2 geometry_msgs metafly_interfaces)
target_link_libraries(controller "${cpp_typesupport_target}" ${LIBSERIAL})

add_executable(teleop_keyboard src/teleop_keyboard.cpp)
ament_target_dependencies(teleop_keyboard rclcpp metafly_interfaces)
target_link_libraries(teleop_keyboard "${cpp_typesupport_target}" ncurses)

add_executable(teleop_ps3 src/teleop_ps3.cpp)
ament_target_dependencies(teleop_ps3 rclcpp metafly_interfaces)
target_link_libraries(teleop_ps3 "${cpp_typesupport_target}")

install(TARGETS
  controller
  teleop_keyboard
  teleop_ps3
  DESTINATION lib/${PROJECT_NAME}
)

# Install launch files.
install(DIRECTORY
  launch
  config
  DESTINATION share/${PROJECT_NAME}/
)

ament_package()
