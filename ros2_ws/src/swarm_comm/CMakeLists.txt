cmake_minimum_required(VERSION 3.8)
project(swarm_comm)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# macOS only: see swarm_msgs/CMakeLists.txt for why -- same Homebrew-Python
# vs. ament-tooling conflict applies here too.
if(APPLE)
  set(Python3_FIND_FRAMEWORK LAST CACHE STRING "" FORCE)
  set(Python3_FIND_STRATEGY LOCATION CACHE STRING "" FORCE)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(swarm_msgs REQUIRED)
find_package(Python3 REQUIRED COMPONENTS Development)

# Explicit RPATH to swarm_msgs' installed lib dir: on macOS, dyld does not
# consistently pick up DYLD_LIBRARY_PATH (set by install/setup.zsh) through
# every process-launch path this binary runs under -- notably, both `nohup`
# and colcon/ament's own `launch_testing` invocation (ctest -> run_test.py)
# strip it before exec'ing the actual node, causing a
# "Library not loaded: @rpath/libswarm_msgs__..." crash that only shows up
# outside of a plain interactive `ros2 run`. Baking the absolute path in
# means the binary never depends on that environment variable surviving.
get_filename_component(SWARM_MSGS_LIB_DIR "${swarm_msgs_DIR}/../../../lib" ABSOLUTE)
list(APPEND CMAKE_INSTALL_RPATH "${SWARM_MSGS_LIB_DIR}")

# MAVSDK (real PX4 flight: Telemetry/Action/Offboard). On macOS, `brew
# install mavsdk` doesn't drop its CMake config on the default search path,
# so point CMake at the Homebrew prefix explicitly. On Linux, building
# MAVSDK from source with `cmake --install` (see MAVSDK's own docs) already
# installs into /usr/local, which CMake searches by default -- no extra
# prefix needed there.
if(APPLE)
  list(APPEND CMAKE_PREFIX_PATH /opt/homebrew/opt/mavsdk)
endif()
find_package(MAVSDK REQUIRED)

add_executable(swarm_node src/swarm_node.cpp)
ament_target_dependencies(swarm_node rclcpp swarm_msgs)
target_link_libraries(swarm_node MAVSDK::mavsdk)

# swarm_msgs exports its rosidl_generator_py library (Python message
# bindings) as part of the same dependency set ament_target_dependencies
# links above -- needed by swarm_dashboard.py / simulate_triangle_flight.py,
# irrelevant to this C++ node. On macOS that .dylib's Python C-API symbols
# (e.g. _PyBool_Type) are built as flat-namespace "resolve from whatever
# process loads me" references, which only works inside an actual Python
# interpreter process. swarm_node isn't one, so loading it crashes at
# startup with "symbol not found in flat namespace '_PyBool_Type'" -- fixed
# by also loading the matching libpython into this process so dyld's flat
# namespace search has somewhere to find those symbols, even though
# swarm_node itself never calls into Python.
target_link_libraries(swarm_node Python3::Python)

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
