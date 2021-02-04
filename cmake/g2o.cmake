find_package(g2o REQUIRED)

include_directories(${G2O_INCLUDE_DIRS})
list(APPEND ALL_TARGET_LIBRARIES ${G2O_LIBS})