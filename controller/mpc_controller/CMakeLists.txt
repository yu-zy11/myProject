project(mpc_controller)
add_library(mpc_controller
  convexMPC.cpp
  convexMPC.h
  math_utils.h
)
target_link_libraries(${PROJECT_NAME}
  PUBLIC
    qpOASES1
)

