cmake_minimum_required(VERSION 3.13.0)
project(mytest CXX)
find_package(common REQUIRED)
set(CMAKE_CXX_STANDARD 14)
set(CMAKE_CXX_STANDARD_REQUIRED true)

if(NOT CMAKE_BUILD_TYPE)
  set(CMAKE_BUILD_TYPE Release)
endif()

set(CMAKE_CXX_FLAGS "-Wall -Wextra")
set(CMAKE_CXX_FLAGS_DEBUG "-g")
set(CMAKE_CXX_FLAGS_RELEASE "-O3")
set(CMAKE_CXX_FLAGS  "${CMAKE_CXX_FLAGS} -march=native")

catkin_package()

set(CMAKE_MODULE_PATH ${CMAKE_MODULE_PATH} "${PROJECT_SOURCE_DIR}/cmake/")

# 指定头文件目录，向编译器指定项目中需要包含的头文件目录
include_directories(${CMAKE_SOURCE_DIR})
include_directories(${CMAKE_CURRENT_SOURCE_DIR})
include_directories(${common_INCLUDE_DIRS})


include_directories(../)
link_libraries(hkust_pl_common)
# add_subdirectory 用于向当前项目添加一个子目录，CMake会在子目录中继续构建项目
add_subdirectory(mylibrary)
# 定义库，并且指定库的名称和源文件
add_library(mytest test.cc)
# 为hdmap指定别名——hdmap::hdmap
# 指定库所需要的链接库
target_link_libraries(mytest
mylibrary
)
