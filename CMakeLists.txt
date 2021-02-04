cmake_minimum_required(VERSION 3.5)
project(pose_graph)
message("start make pose_graph")

# 一定要加-march=native， 不然使用pcl的一些函数会报错
#add_definitions(-DCPP11)
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++17 -O3 -march=native")

list(APPEND CMAKE_MODULE_PATH ${PROJECT_SOURCE_DIR}/cmake_modules)

set(ALL_TARGET_LIBRARIES "")
include(cmake/global_defination.cmake)
include(cmake/g2o.cmake)
include(cmake/glog.cmake)


include_directories(
        ${PROJECT_SOURCE_DIR}
        ${PROJECT_SOURCE_DIR}/include
        ${catkin_INCLUDE_DIRS}
)


file(GLOB_RECURSE ALL_SRCS "src/*.cpp")
file(GLOB_RECURSE THIRD_PARTY_SRCS "Thirdparty/*.cpp")
#list(REMOVE_ITEM ALL_SRCS ${THIRD_PARTY_SRCS})


# lidar slam and localize
add_executable(unary_edge exec/unary_edge.cpp ${ALL_SRCS})
target_link_libraries(unary_edge ${ALL_TARGET_LIBRARIES})

