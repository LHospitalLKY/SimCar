find_package(OpenCV 3.4 REQUIRED)

add_library(Quad_GradientDesent
    include/CostFunction.h
    include/Quad_CostFunction.h
    include/GradientDesent.h
    include/ResidualFunction.h
    src/ResidualFunction.cpp
    src/GradientDesent.cpp
    src/Quad_CostFunction.cc
)
target_link_libraries(Quad_GradientDesent
    ${OpenCV_LIBS}
)

add_executable(Quad_CostFunction_TEST
    src/Quad_CostFunction_TEST.cpp
)
target_link_libraries(Quad_CostFunction_TEST
    Quad_GradientDesent
    ${OpenCV_LIBS}
)