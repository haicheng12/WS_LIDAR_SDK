cmake_minimum_required(VERSION 3.5 FATAL_ERROR)

project(ws30_pcd_et3)

# 设置 C++ 标准
set(CMAKE_CXX_STANDARD 11)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# 查找 pthread 库
find_library(PTHREAD_LIB pthread PATHS "/usr/lib/x86_64-linux-gnu")

# 检查是否找到了 pthread 库
if(NOT PTHREAD_LIB)
  message(FATAL_ERROR "Could not find the pthread library")
endif()

find_package(PCL 1.10 REQUIRED)

include_directories(
  include
  ${PCL_INCLUDE_DIRS}
)

link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})

add_executable (scan_frame 
src/mission/mission.cpp
src/task/task.cpp
src/task/points_task.cpp
src/task/imu_task.cpp
src/task/scan_task.cpp
src/udp/udp.cpp
src/main.cpp)

# 链接 pthread 库
target_link_libraries (scan_frame ${PCL_LIBRARIES} ${PTHREAD_LIB})

# 添加编译器标志
target_compile_options(scan_frame PUBLIC "-pthread")

# 如果需要，添加头文件搜索路径
include_directories("/usr/include")