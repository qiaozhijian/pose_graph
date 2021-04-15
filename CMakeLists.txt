cmake_minimum_required(VERSION 3.5)
project(pose_graph)
message("start make pose_graph")

# -march=native这个参数需谨慎
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++17 -O3 ")
set(CMAKE_CXX_STANDARD 17)

list(APPEND CMAKE_MODULE_PATH ${PROJECT_SOURCE_DIR}/cmake_modules)

set(ALL_TARGET_LIBRARIES "")
include(cmake/global_defination.cmake)
include(cmake/g2o.cmake)
include(cmake/glog.cmake)
#include(cmake/OpenCV.cmake)


include_directories(
        ${PROJECT_SOURCE_DIR}
        ${PROJECT_SOURCE_DIR}/include
        ${catkin_INCLUDE_DIRS}
)


file(GLOB_RECURSE ALL_SRCS "src/*.cpp")
file(GLOB_RECURSE THIRD_PARTY_SRCS "Thirdparty/*.cpp")
#list(REMOVE_ITEM ALL_SRCS ${THIRD_PARTY_SRCS})


#lidar slam and localize
add_executable(unary_edge exec/unary_edge.cpp ${ALL_SRCS})
target_link_libraries(unary_edge ${ALL_TARGET_LIBRARIES})

#RobustOptimize
add_executable(RobustOptimize exec/RobustTest.cpp ${ALL_SRCS})
target_link_libraries(RobustOptimize ${ALL_TARGET_LIBRARIES})




