add_library(augmented_lagrangian
        al_cost.hpp
        al_cost.cpp
        al_problem.hpp
        al_problem.cpp
        al_solver.hpp
        al_solver.cpp
        )

target_link_libraries(augmented_lagrangian
        PUBLIC
        fmt::fmt
        PRIVATE
        constraints
        problem
        ilqr
        utils
        )

add_library(altrocpp::augmented_lagrangian ALIAS augmented_lagrangian)