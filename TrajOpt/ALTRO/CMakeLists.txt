# Add subdirectories
#add_subdirectory(eigen)
list(APPEND CMAKE_PREFIX_PATH
        "TrajOpt/ALTRO/third_party/Eigen_3_4_0"
)
find_package(Eigen3 3.4.0 REQUIRED)
include_directories(${EIGEN3_INCLUDE_DIRS})

add_subdirectory(fmt)
add_subdirectory(altrocpp)
add_subdirectory(linesearch)
add_subdirectory(tvlqr)

# Create imported target altrocpp::altro
add_library(altrocpp::altro INTERFACE IMPORTED)

# Set properties for altrocpp::altro
target_include_directories(altrocpp::altro INTERFACE "${CMAKE_CURRENT_SOURCE_DIR}/altrocpp")
target_link_libraries(altrocpp::altro INTERFACE
        fmt::fmt
        altrocpp::augmented_lagrangian
        altrocpp::common
        altrocpp::constraints
        altrocpp::ilqr
        altrocpp::problem
        altrocpp::utils
        altrocpp::threadpool
)

# Add other subdirectories
add_subdirectory(altrocpp_interface)
add_subdirectory(altro)