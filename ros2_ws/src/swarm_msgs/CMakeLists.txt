cmake_minimum_required(VERSION 3.8)
project(swarm_msgs)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# macOS only: CMake's Python3 detection defaults to searching Frameworks
# before PATH. Homebrew's python@3.14 framework on this machine has a
# broken pyexpat (incompatible with the system libexpat.1.dylib -- "Symbol
# not found: _XML_SetAllocTrackerActivationThreshold"), which crashes cmake
# configure the moment ament's build-time tooling (package_xml_2_cmake.py,
# invoked by rosidl_generate_interfaces below) tries to run a Python script.
# Force PATH-first detection so it picks up the ROS/conda toolchain's own
# Python instead.
if(APPLE)
  set(Python3_FIND_FRAMEWORK LAST CACHE STRING "" FORCE)
  set(Python3_FIND_STRATEGY LOCATION CACHE STRING "" FORCE)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rosidl_default_generators REQUIRED)

rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/SwarmStatus.msg"
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
