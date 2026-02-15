cmake_minimum_required(VERSION 3.0.2)
project(ocs2_ddp)

find_package(Boost 1.71.0 REQUIRED COMPONENTS
  system
  filesystem
)

# Generate compile_commands.json for clang tools
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)


###########
## Build ##
###########

add_library(${PROJECT_NAME}
  src/riccati_equations/ContinuousTimeRiccatiEquations.cpp
  src/riccati_equations/DiscreteTimeRiccatiEquations.cpp
  src/riccati_equations/RiccatiModification.cpp
  src/search_strategy/LevenbergMarquardtStrategy.cpp
  src/search_strategy/LineSearchStrategy.cpp
  src/search_strategy/StrategySettings.cpp
  src/ContinuousTimeLqr.cpp
  src/GaussNewtonDDP.cpp
  src/HessianCorrection.cpp
  src/ILQR.cpp
  src/SLQ.cpp
  src/DDP_Settings.cpp
  src/DDP_HelperFunctions.cpp
)
target_link_libraries(${PROJECT_NAME} PUBLIC
  ocs2_core
  ocs2_oc
  ocs2_mpc
)
target_include_directories(${PROJECT_NAME} PUBLIC include)
target_compile_options(${PROJECT_NAME} PUBLIC ${OCS2_CXX_FLAGS})