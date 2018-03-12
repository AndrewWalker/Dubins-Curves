extern "C" {
#include "dubins.h"
}

#include "gtest/gtest.h"

struct Inputs
{
    DubinsPathType word;
    double a;
    double b;
    double d;
};

struct Outputs
{
    int errcode;
    double params[3];
    double length;
};

struct Param
{
    Param(DubinsPathType, double a, double b, double d, int code, double p1, double p2, double p3, double len)
    {
    }

    Inputs inputs;
    Outputs outputs;
};

Param params[] = {
    Param(LSL, -3.141593,-3.141593, 0.400000, 0, 3.141593, 0.4, 3.14592, 6.68),
    Param(LSR, -3.141593,-3.141593, 0.400000, 0, 5.888394,0.400001,5.888394, 12.18)
};


class MonteCarloTests : public ::testing::TestWithParam<Param>
{
public:
    void SetUp()
    {
        turning_radius = 1.0;
        q0[0] = 0.0;
        q0[1] = 0.0;
        q0[2] = GetParam().inputs.a;
        q1[0] = GetParam().inputs.b;
        q1[1] = 0.0;
        q1[2] = GetParam().inputs.d;
    }

protected:
    double turning_radius;
    double q0[3];
    double q1[3];
};

TEST_P(MonteCarloTests, Simple)
{
    DubinsPath path;
    int code = dubins_path(&path, q0, q1, turning_radius, GetParam().inputs.word);
    ASSERT_EQ(code, GetParam().outputs.errcode);
    double len = dubins_path_length(&path);
    ASSERT_DOUBLE_EQ(len, GetParam().outputs.length);
    for(int i = 0; i < 3; i++) {
        ASSERT_DOUBLE_EQ(path.param[i], GetParam().outputs.params[i]);
    }
}

INSTANTIATE_TEST_CASE_P(Simple,
                        MonteCarloTests,
                        ::testing::ValuesIn(params));

