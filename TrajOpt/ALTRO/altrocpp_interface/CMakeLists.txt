add_library(altrocpp_interface STATIC
        altrocpp_interface.hpp
        altrocpp_interface.cpp
)
target_link_libraries(altrocpp_interface
        PUBLIC
        altro::altro
        altrocpp::altro
        Eigen3::Eigen

        PRIVATE
        fmt::fmt
)
target_include_directories(altrocpp_interface
        PRIVATE
        $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/..>
)

add_library(altrocpp::interface ALIAS altrocpp_interface)