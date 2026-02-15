cmake_minimum_required(VERSION 3.20)
project(planning)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -w")
set(CMAKE_POSITION_INDEPENDENT_CODE ON)
add_definitions(-DBOOST_BIND_GLOBAL_PLACEHOLDERS)

if (NOT DEFINED PLANNING_DEBUG)
    set(PLANNING_DEBUG 3)
endif ()
add_definitions(-DPLANNING_DEBUG=${PLANNING_DEBUG})

find_package(catkin REQUIRED COMPONENTS
        nav_msgs
        roscpp
        rospy
        std_msgs
        tf2
        tf2_ros
        #tf2_geometry_msgs
        # jsk_rviz_plugins
)

# -------------------------------------------------------------
# Hack: Remove system Boost explicit paths (1.71.0) from catkin_LIBRARIES
# to avoid conflict with our local Boost 1.83.0
set(TEMP_LIBS "")
foreach(lib ${catkin_LIBRARIES})
    if(lib MATCHES "libboost_.*.so.1.71.0")
        message(STATUS "Removing interfering system boost lib from catkin: ${lib}")
    else()
        list(APPEND TEMP_LIBS ${lib})
    endif()
endforeach()
set(catkin_LIBRARIES ${TEMP_LIBS})
# -------------------------------------------------------------

list(APPEND CMAKE_PREFIX_PATH
        "Thirdparty/boost_1_83_0"
        "Thirdparty/nlohmann_3_11_2"
        "Thirdparty/spdlog_1_12_0"
        "Thirdparty/pugixml_1_14"
        "Thirdparty/Eigen_3_4_0"
)
find_package(Eigen3 3.4.0 REQUIRED)
include_directories(${EIGEN3_INCLUDE_DIRS})
find_package(Boost 1.83.0 REQUIRED COMPONENTS system filesystem)
find_package(Python3 3.8 COMPONENTS Interpreter Development NumPy REQUIRED)
find_package(nlohmann_json 3.11.2 REQUIRED)
find_package(spdlog 1.12.0 REQUIRED)
find_package(pugixml 1.14 REQUIRED)


add_subdirectory(ALTRO)
#add_subdirectory(OCS2)

include_directories(
        ${CMAKE_SOURCE_DIR}
        ${catkin_INCLUDE_DIRS}
)

add_executable(${PROJECT_NAME}
        main.cpp
        Cost/cost.cpp
        Common/types.cpp
        Model/model.cpp
        Model/integrator.cpp
        Map/discrete_points_math.cc
        Interfaces/planner_node.cpp
        Interfaces/solver_interface.cpp
        Visualization/visualization.cpp
        Prediction/prediction_trajectory.cpp

        PathSegment/aabox2d.cc
        PathSegment/box2d.cc
        PathSegment/line_segment2d.cc
        PathSegment/math_utils.cc
        PathSegment/path.cc
        PathSegment/polygon2d.cc
        PathSegment/vec2d.cc

        TrafficRules/traffic_light.cc
        TrafficRules/crosswalk.cpp
        TrafficRules/vehicle_speed_decider.cpp
        # SamplingTrajectory/mcts.h
)

target_link_libraries(${PROJECT_NAME}
        PRIVATE
        #        ocs2
        pugixml::pugixml
        altro::altro
        spdlog::spdlog
        Python3::Python
        Python3::NumPy
        nlohmann_json::nlohmann_json
        ${Boost_LIBRARIES}
        ${catkin_LIBRARIES}
)
target_include_directories(${PROJECT_NAME} PUBLIC /usr/include/python3.8)