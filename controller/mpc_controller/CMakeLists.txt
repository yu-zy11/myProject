project(mpc_controller)
add_library(${PROJECT_NAME}
  convexMPC.cpp
  convexMPC.h
  math_utils.h
)
message("build "${PROJECT_NAME})
target_link_libraries(${PROJECT_NAME}
  PUBLIC
    qpOASES
)
