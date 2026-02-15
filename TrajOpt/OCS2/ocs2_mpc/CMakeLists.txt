cmake_minimum_required(VERSION 3.0.2)
project(ocs2_mpc)

find_package(Boost 1.71.0 REQUIRED COMPONENTS
  system
  filesystem
)

# Generate compile_commands.json for clang tools
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)


add_library(${PROJECT_NAME}
  src/LoopshapingSystemObservation.cpp
  src/MPC_BASE.cpp
  src/MPC_Settings.cpp
  src/SystemObservation.cpp
  src/MRT_BASE.cpp
  src/MPC_MRT_Interface.cpp
  # src/MPC_OCS2.cpp
)
target_link_libraries(${PROJECT_NAME}
  ocs2_core
  ocs2_oc
)
target_include_directories(${PROJECT_NAME} PUBLIC include)
target_compile_options(${PROJECT_NAME} PUBLIC ${OCS2_CXX_FLAGS})