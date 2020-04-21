#include <chrono>

constexpr bool DEBUG_PRINT_RESULTS = true;

// clang-format off
#define TIME_IT(name, a) \
    if (DEBUG_PRINT_RESULTS) { \
        auto start = std::chrono::high_resolution_clock::now(); \
        a; \
        auto elapsed = std::chrono::high_resolution_clock::now() - start; \
        std::cout << name \
                  << ": " \
                  << std::chrono::duration_cast<std::chrono::nanoseconds>(elapsed).count() \
                     / 1000000.0f << " ms" << std::endl; \
    } else { \
        a; \
    }
// clang-format on