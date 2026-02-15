cmake_minimum_required(VERSION 3.8)
project(path_segment VERSION 0.1.0)
include_directories(
        ${PROJECT_SOURCE_DIR}/include
)
file(GLOB SOURCES "*.cc")
add_library(${PROJECT_NAME} INTERFACE ${SOURCES})

target_include_directories(${PROJECT_NAME} INTERFACE ${CMAKE_CURRENT_SOURCE_DIR})