set(HEADERS
    BandedLDL.h
    BumpFunction.h
    DampedOptimizer.h
    Frenet.h
    MatrixUtil.h
    NumericalDerivative.h
    ScalarFunctionWrappers.h
    TrajectoryOptimizer.h

    cost/ActorAvoidanceCost.h
    cost/BumpCost.h
    cost/FunctionOfFilterCost.h
    cost/LateralCost.h
    cost/LimitCosts.h
    cost/PriorCost.h
    cost/ProgressCost.h
    cost/SmoothnessCost.h
)

set(SOURCES
    TrajectoryOptimizer.cpp

)

add_library(TrajectoryOptimizer SHARED
    ${HEADERS}
    ${SOURCES}
)

# not using gtest now
add_executable(TrajectoryOptimizerTests 
    tests/TestSuite.cpp
)