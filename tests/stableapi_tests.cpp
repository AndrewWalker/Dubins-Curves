extern "C" {
#include "dubins.h"
}

#include "gtest/gtest.h"

class DubinsTests : public ::testing::Test
{
public:
       
    void SetUp() 
    {
        turning_radius = 1.0;
        configure_inputs(0.0, 0.0, 1.0);
    }

    void configure_inputs(double a, double b, double d)
    {
        q0[0] = 0.0;
        q0[1] = 0.0;
        q0[2] = a;
        q1[0] = d;
        q1[1] = 0.0;
        q1[2] = b;
    }

protected:
    double turning_radius;
    double q0[3];
    double q1[3];
};

TEST_F(DubinsTests, shortestPath)
{
    // find the shortest path
    DubinsPath path;
    int err = dubins_shortest_path(&path, q0, q1, turning_radius);
    ASSERT_EQ(err, EDUBOK);
}

TEST_F(DubinsTests, invalidTurningRadius)
{
    // find the shortest path
    DubinsPath path;
    int err = dubins_shortest_path(&path, q0, q1, -1.0);
    ASSERT_EQ(err, EDUBBADRHO);
}

TEST_F(DubinsTests, noPath)
{
    configure_inputs(0.0, 0.0, 10.0);
    
    // find the shortest path
    DubinsPath path;
    int err = dubins_path(&path, q0, q1, turning_radius, LRL);
    ASSERT_EQ(err, EDUBNOPATH);
}

TEST_F(DubinsTests, pathLength)
{
    configure_inputs(0.0, 0.0, 4.0);

    // find the shortest path
    DubinsPath path;
    dubins_shortest_path(&path, q0, q1, turning_radius);
    double res = dubins_path_length(&path);
    ASSERT_DOUBLE_EQ(res, 4.0);
}

TEST_F(DubinsTests, simplePath)
{
    // find the parameters for a single Dubin's word
    DubinsPath path;
    int err = dubins_path(&path, q0, q1, turning_radius, LSL);
    ASSERT_EQ(err, EDUBOK);
}

TEST_F(DubinsTests, segmentLengths)
{
    configure_inputs(0.0, 0.0, 4.0);

    // find the parameters for a single Dubin's word
    DubinsPath path;
    int err = dubins_path(&path, q0, q1, turning_radius, LSL);
    ASSERT_EQ(err, EDUBOK);
    ASSERT_DOUBLE_EQ(dubins_segment_length(&path, 0), 0.0);
    ASSERT_DOUBLE_EQ(dubins_segment_length(&path, 1), 4.0);
    ASSERT_DOUBLE_EQ(dubins_segment_length(&path, 2), 0.0);

}

TEST_F(DubinsTests, sample)
{
    configure_inputs(0.0, 0.0, 4.0);

    // find the parameters for a single Dubin's word
    DubinsPath path;
    int err = dubins_path(&path, q0, q1, turning_radius, LSL);
    ASSERT_EQ(err, EDUBOK);

    double qsamp[3];
    err = dubins_path_sample(&path, 0.0, qsamp);
    ASSERT_EQ(err, EDUBOK);
    ASSERT_DOUBLE_EQ(qsamp[0], q0[0]);
    ASSERT_DOUBLE_EQ(qsamp[1], q0[1]);
    ASSERT_DOUBLE_EQ(qsamp[2], q0[2]);

    err = dubins_path_sample(&path, 4.0, qsamp);
    ASSERT_EQ(err, EDUBOK);
    ASSERT_DOUBLE_EQ(qsamp[0], q1[0]);
    ASSERT_DOUBLE_EQ(qsamp[1], q1[1]);
    ASSERT_DOUBLE_EQ(qsamp[2], q1[2]);
}

TEST_F(DubinsTests, sampleOutOfBounds)
{
    configure_inputs(0.0, 0.0, 4.0);

    // find the parameters for a single Dubin's word
    DubinsPath path;
    int err = dubins_path(&path, q0, q1, turning_radius, LSL);
    ASSERT_EQ(err, EDUBOK);

    double qsamp[3];
    err = dubins_path_sample(&path, -1.0, qsamp);
    ASSERT_EQ(err, EDUBPARAM);
    err = dubins_path_sample(&path, 5.0, qsamp);
    ASSERT_EQ(err, EDUBPARAM);
}

int nop_callback(double q[3], double t, void* data) 
{
    return 0;
}

TEST_F(DubinsTests, sampleMany)
{
    configure_inputs(0.0, 0.0, 4.0);

    // find the parameters for a single Dubin's word
    DubinsPath path;
    int err = dubins_path(&path, q0, q1, turning_radius, LSL);
    ASSERT_EQ(err, EDUBOK);
    err = dubins_path_sample_many(&path, 1.0, &nop_callback, NULL);
}


TEST_F(DubinsTests, pathType)
{
    // find the parameters for a single Dubin's word
    DubinsPath path;

    for(int i = 0; i < 6; i++) {
        DubinsPathType t = (DubinsPathType)i;
        int err = dubins_path(&path, q0, q1, turning_radius, t);
        if( err == EDUBOK ) {
            ASSERT_EQ(t, dubins_path_type(&path));
        }
    }
}

TEST_F(DubinsTests, endPoint)
{
    configure_inputs(0.0, 0.0, 4.0);

    // find the parameters for a single Dubin's word
    DubinsPath path;
    int err = dubins_path(&path, q0, q1, turning_radius, LSL);
    ASSERT_EQ(err, EDUBOK);

    double qsamp[3];
    err = dubins_path_endpoint(&path, qsamp);
    ASSERT_EQ(err, EDUBOK);
    ASSERT_NEAR(qsamp[0], q1[0], 1e-8);
    ASSERT_NEAR(qsamp[1], q1[1], 1e-8);
    ASSERT_NEAR(qsamp[2], q1[2], 1e-8);
}


int main(int argc, char **argv) 
{
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
