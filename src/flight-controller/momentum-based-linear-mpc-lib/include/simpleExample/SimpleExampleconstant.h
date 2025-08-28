#ifndef CONSTANT_SIMPLE_EXAMPLE_H
#define CONSTANT_SIMPLE_EXAMPLE_H

#include <array>

#define N_U_1 1
#define N_Y_1 1
#define N_U_2 1
#define N_Y_2 1

constexpr std::array<int, 2> xIdx = {0, 1};

constexpr std::array<int, N_Y_2> createY2Idx()
{
    std::array<int, N_Y_2> temp = {};
    for (int i = 0; i < N_Y_2; ++i)
    {
        temp[i] = i;
    }
    return temp;
}

constexpr std::array<int, N_Y_2> Y2Idx = createY2Idx();

constexpr std::array<int, N_U_1> createU1Idx()
{
    std::array<int, N_U_1> temp = {};
    for (int i = 0; i < N_U_1; ++i)
    {
        temp[i] = i + N_Y_2;
    }
    return temp;
}

constexpr std::array<int, N_U_1> U1Idx = createU1Idx();

constexpr std::array<int, N_U_2> createU2Idx()
{
    std::array<int, N_U_2> temp = {};
    for (int i = 0; i < N_U_2; ++i)
    {
        temp[i] = i + N_U_1 + N_Y_2;
    }
    return temp;
}

constexpr std::array<int, N_U_2> U2Idx = createU2Idx();

#endif // CONSTANT_SIMPLE_EXAMPLE_H
