add_executable(run_ceres
    test_ceres.cpp
)
target_link_libraries(run_ceres
    ${CERES_LIBRARIES}
)


add_executable(run_g2o
    test_g2o.cpp
)
