cmake_minimum_required(VERSION 3.8)
project(swarm_comm)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(swarm_msgs REQUIRED)

# MAVSDK (real PX4 flight: Telemetry/Action/Offboard) -- installed via
# `brew install mavsdk`, which doesn't drop its CMake config on the default
# search path, so point CMake at the Homebrew prefix explicitly.
list(APPEND CMAKE_PREFIX_PATH /opt/homebrew/opt/mavsdk)
find_package(MAVSDK REQUIRED)

add_executable(swarm_node src/swarm_node.cpp)
ament_target_dependencies(swarm_node rclcpp swarm_msgs)
target_link_libraries(swarm_node MAVSDK::mavsdk)

install(TARGETS
  swarm_node
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY
  launch
  DESTINATION share/${PROJECT_NAME}
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

  find_package(launch_testing_ament_cmake REQUIRED)
  add_launch_test(test/test_election.py)
endif()

ament_package()
