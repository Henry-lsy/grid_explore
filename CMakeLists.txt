cmake_minimum_required(VERSION 3.0.2)
project(grid_explorer)


# 设置C++标准
set(CMAKE_CXX_STANDARD 14)
set(CMAKE_CXX_STANDARD_REQUIRED True)

set(CMAKE_BUILD_TYPE Release)


## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  roscpp
  visualization_msgs
)

catkin_package(
 INCLUDE_DIRS include
#  LIBRARIES grid_explorer
 CATKIN_DEPENDS roscpp visualization_msgs
#  DEPENDS system_lib
)

###########
## Build ##
###########

## Specify additional locations of header files
## Your package locations should be listed before other locations
include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

# 添加主程序可执行文件
add_executable(grid_explorer_node src/grid_explorer_node.cpp src/grid_partitioning.cpp)
target_link_libraries(grid_explorer_node ${catkin_LIBRARIES})
# 添加选项来选择是否编译测试用例
# option(BUILD_TESTS "Build the test cases" OFF)

# if(BUILD_TESTS)
#     message(STATUS "Building test cases")
#     add_executable(TSP_Grid_Tests tests.cpp src/grid_partitioning.cpp)
#     # 可以在这里为测试用例定义不同的预处理宏或编译选项
#     target_compile_definitions(TSP_Grid_Tests PRIVATE RUN_TESTS=1)
# else()
#     message(STATUS "Skipping test cases")
# endif()


