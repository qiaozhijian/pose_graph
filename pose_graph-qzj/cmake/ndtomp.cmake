add_subdirectory(Thirdparty/ndt_omp)
add_definitions(-DUSE_NDT_OMP)
include_directories(
        ${PROJECT_SOURCE_DIR}/Thirdparty/ndt_omp/include
)
list(APPEND ALL_TARGET_LIBRARIES ${PROJECT_SOURCE_DIR}/Thirdparty/ndt_omp/lib/libndt_omp.so)
