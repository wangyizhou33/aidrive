#include "ActorAvoidanceCostTest.h"
#include "BandedLDLTest.h"
#include "BumpFunctionTest.h"
#include "DampedOptimizerTest.h"
#include "FrenetTest.h"
#include "LateralCostTest.h"
#include "LimitCostsTest.h"
#include "MatrixUtilTest.h"
#include "NumericalDerivativeTest.h"
#include "PriorCostTest.h"
#include "ProgressCostTest.h"
#include "ScalarFunctionWrappersTest.h"
#include "SmoothBarrierFunctionTest.h"
#include "SmoothnessCostTest.h"
#include "TrajectoryOptimizerTest.h"
#include <iostream>

void prettyPrint(bool in) // true if pass, false if fail
{
    if (!in)
        std::cout << "\033[1;31m\t...Failed\033[0m\n" << std::endl;
    else
        std::cout << "\033[1;32m\t...Passed\033[0m\n" << std::endl;
}

int main()
{
    std::cout << "Running the test suite:" << std::endl;
    std::cout << "=======================" << std::endl;

    std::cout << "1. Test MatrixUtil..." << std::endl;
    prettyPrint(TestMatrixUtil());

    std::cout << "2. Test BandedLDL..." << std::endl;
    prettyPrint(TestBandedLDL());

    std::cout << "3. Test DampedOptimizer..." << std::endl;
    prettyPrint(TestDampedOptimizer());

    std::cout << "4. Test SmoothBarrierFunction..." << std::endl;
    prettyPrint(TestSmoothBarrierFunction());

    std::cout << "5. Test NumericalDerivative..." << std::endl;
    prettyPrint(TestNumericalDerivative());

    std::cout << "6. Test BumpFunction..." << std::endl;
    prettyPrint(TestBumpFunction());

    std::cout << "7. Test ScalarFunctionWrappers..." << std::endl;
    prettyPrint(TestScalarFunctionWrappers());

    std::cout << "8. Test ProgressCost..." << std::endl;
    prettyPrint(TestProgressCost());

    std::cout << "9. Test PriorCost..." << std::endl;
    prettyPrint(TestPriorCost());

    std::cout << "9. Test SmoothnessCost..." << std::endl;
    prettyPrint(TestSmoothnessCost());

    std::cout << "10. Test LimitCost..." << std::endl;
    prettyPrint(TestLimitCost());

    std::cout << "11. Test LateralCost..." << std::endl;
    prettyPrint(TestLateralCost());

    std::cout << "12. Test Frenet..." << std::endl;
    prettyPrint(TestFrenet());

    std::cout << "13. Test ActorAvoidance..." << std::endl;
    prettyPrint(TestActorAvoidanceCost());

    std::cout << "14. Test optimizer..." << std::endl;
    prettyPrint(TestTrajectoryOptimizer());

    std::cout << "\nAll tests passed." << std::endl;
    return 0;
}