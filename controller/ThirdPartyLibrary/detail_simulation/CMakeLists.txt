# 指定CMake版本
cmake_minimum_required(VERSION 3.0)
 
# 判断编译器类型,如果是gcc编译器,则在编译选项中加入c++11支持
if(CMAKE_COMPILER_IS_GNUCC)
    message("COMPILER IS GNUCC")
    add_definitions(-std=c++11 )
endif(CMAKE_COMPILER_IS_GNUCC)

project(detail_simulation)
 
# C++ 调试编译时使用的标志
# set(CMAKE_CXX_FLAGS_DEBUG "$ENV{CXXFLAGS} -O0 -Wall -ggdb3")
# # C++ 发行编译时使用的标志
# set(CMAKE_CXX_FLAGS_RELEASE "$ENV{CXXFLAGS} -O3 -Wall")
 
# 加载并处理一个子文件夹下的 CMakeLists.txt 文件

message(${CMAKE_CURRENT_SOURCE_DIR})
message(${PROJECT_NAME} )


aux_source_directory(. SRC_LIST_DETAIL_SIMULATION) # 搜索当前目录下的所有.cpp文件
message(${SRC_LIST_DETAIL_SIMULATION})
add_library(${PROJECT_NAME}  ${SRC_LIST_DETAIL_SIMULATION})

target_link_libraries(detail_simulation PUBLIC linearDynamics)

add_subdirectory(${CMAKE_CURRENT_SOURCE_DIR}/linearDynamics)