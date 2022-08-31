cmake_minimum_required(VERSION 3.0)
set(CMAKE_BUILD_TYPE "Debug")

project(main)

link_libraries(linearDynamics)

add_executable(${PROJECT_NAME} main.cpp)
find_package(Threads)
target_link_libraries(${PROJECT_NAME} ${CMAKE_THREAD_LIBS_INIT})

# link_libraries(linearDynamics)
# target_link_libraries(${PROJECT_NAME} linearDynamics)

add_subdirectory(controller)
