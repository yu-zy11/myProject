# 指定CMake版本
cmake_minimum_required(VERSION 3.0)
 
# 判断编译器类型,如果是gcc编译器,则在编译选项中加入c++11支持
if(CMAKE_COMPILER_IS_GNUCC)
    message("COMPILER IS GNUCC")
    add_definitions(-std=c++11 )
endif(CMAKE_COMPILER_IS_GNUCC)

project(linearDynamics)

message(${CMAKE_CURRENT_SOURCE_DIR})
message(${PROJECT_NAME} )

aux_source_directory(. SRC_LIST_LINEAR_DYNAMICS) # 搜索当前目录下的所有.cpp文件
message("tets")
add_library(${PROJECT_NAME}  ${SRC_LIST_LINEAR_DYNAMICS})