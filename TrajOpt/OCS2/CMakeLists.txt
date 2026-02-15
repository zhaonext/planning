cmake_minimum_required(VERSION 3.26)
project(ocs2test)

set(CMAKE_CXX_STANDARD 17)

add_subdirectory(ocs2_core)
add_subdirectory(ocs2_oc)
add_subdirectory(ocs2_mpc)
add_subdirectory(ocs2_ddp)

target_include_directories(ocs2_core PUBLIC ocs2_thirdparty/include)

#add_executable(ocs2test main.cpp)
add_library(ocs2  INTERFACE)
target_link_libraries(ocs2  INTERFACE ocs2_ddp)
