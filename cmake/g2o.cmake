if (POLICY CMP0072)
    set(OpenGL_GL_PREFERENCE LEGACY)
endif()


find_package(G2O REQUIRED)
include_directories(${G2O_INCLUDE_DIR})

# csparse
find_package(CSparse REQUIRED)
include_directories(${CSPARSE_INCLUDE_DIR})

# csparse
find_package(Cholmod REQUIRED)
include_directories(${CHOLMOD_INCLUDE_DIR})


list(APPEND ALL_TARGET_LIBRARIES
        ${CSPARSE_LIBRARY}
        ${CHOLMOD_LIBRARY}
        g2o_cli
        g2o_solver_slam2d_linear g2o_types_icp g2o_types_slam2d
        g2o_core g2o_interface g2o_solver_csparse g2o_solver_structure_only
        g2o_types_sba g2o_types_slam3d g2o_csparse_extension
        g2o_solver_dense g2o_stuff
        g2o_types_sclam2d g2o_parser g2o_solver_pcg
        g2o_types_data g2o_types_sim3
        )