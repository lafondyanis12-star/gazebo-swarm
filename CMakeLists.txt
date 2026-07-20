cmake_minimum_required(VERSION 3.16)
project(swarm_node CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

find_package(MAVSDK REQUIRED)
find_package(Threads REQUIRED)

add_executable(swarm_node swarm_node.cpp)
target_link_libraries(swarm_node PRIVATE MAVSDK::mavsdk Threads::Threads)
