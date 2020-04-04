add_executable(run_ceres
    test_ceres.cpp
    ${OpenCV_LIBS}
)

MESSAGE(STATUS "libs: ${OpenCV_LIBS}")

add_executable(run_g2o
    test_g2o.cpp
    ${OpenCV_LIBS}
)
