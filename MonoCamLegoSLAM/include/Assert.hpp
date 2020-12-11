# pragma once

#include <iostream>

// Taken from https://stackoverflow.com/a/11994395/5023438
#define FE_0(WHAT)
#define FE_1(WHAT, X) WHAT(X) 
#define FE_2(WHAT, X, ...) WHAT(X)FE_1(WHAT, __VA_ARGS__)
#define FE_3(WHAT, X, ...) WHAT(X)FE_2(WHAT, __VA_ARGS__)
#define FE_4(WHAT, X, ...) WHAT(X)FE_3(WHAT, __VA_ARGS__)
#define FE_5(WHAT, X, ...) WHAT(X)FE_4(WHAT, __VA_ARGS__)
#define FE_6(WHAT, X, ...) WHAT(X)FE_5(WHAT, __VA_ARGS__)
#define FE_7(WHAT, X, ...) WHAT(X)FE_6(WHAT, __VA_ARGS__)
#define FE_8(WHAT, X, ...) WHAT(X)FE_7(WHAT, __VA_ARGS__)
#define FE_9(WHAT, X, ...) WHAT(X)FE_8(WHAT, __VA_ARGS__)

#define GET_MACRO(_0,_1,_2,_3,_4,_5,_6,_7,_8,_9,NAME,...) NAME 
#define FOR_EACH(action,...) \
  GET_MACRO(_0,__VA_ARGS__,FE_9,FE_8,FE_7,FE_6,FE_5,FE_4,FE_3,FE_2,FE_1,FE_0)(action,__VA_ARGS__)

#define PRINT(X) std::cout << "  " #X ":\n    " << (X) << '\n';
#define PRINTALL(...) FOR_EACH(PRINT,__VA_ARGS__)

/**
 * assert(expressionConvertibleToBool)
 * assertWithContext(expressionConvertibleToBool, relevantExpressionsToPrint)
 * Usage:
 *     assert(i != 0)
 *     assert(ptr)
 *     assertWithContext(x + y % 2 == 0, x, y)
 *     assertWithContext(vec.size() >= 10, vec.size())
 */
#define assert(expr, ...) \
do { \
    if (!static_cast<bool>(expr)) { \
        std::cerr << "\nAssertion failure!\n" __FILE__ "\n" << __LINE__ << ": " #expr "\n\n"; \
        std::exit(-1); \
    } \
} while (false);

#define assertWithContext(expr, ...) \
do { \
    if (!static_cast<bool>(expr)) { \
        std::cerr << "\nAssertion failure!\n" __FILE__ "\n" << __LINE__ << ": " #expr "\n"; \
        PRINTALL(__VA_ARGS__) \
        std::cout << '\n'; \
        std::exit(-1); \
    } \
} while (false);
