// Copyright (c) 2008-2010, Andrew Walker
// 
// Permission is hereby granted, free of charge, to any person obtaining a copy
// of this software and associated documentation files (the "Software"), to deal
// in the Software without restriction, including without limitation the rights
// to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
// copies of the Software, and to permit persons to whom the Software is
// furnished to do so, subject to the following conditions:
// 
// The above copyright notice and this permission notice shall be included in
// all copies or substantial portions of the Software.
// 
// THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
// IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
// FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
// AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
// LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
// OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN
// THE SOFTWARE.

#include "dubins.h"
#include <cmath>

#define LSL (0)
#define LSR (1)
#define RSL (2)
#define RSR (3)
#define RLR (4)
#define LRL (5)

#define UNPACK_INPUTS(alpha, beta)       \
    double sa = ::sin(alpha);            \
    double sb = ::sin(beta);             \
    double ca = ::cos(alpha);            \
    double cb = ::cos(beta);             \
    double c_ab = ::cos(alpha - beta);   \
    
#define PACK_OUTPUTS(outputs)       \
    outputs[0]  = t;                \
    outputs[1]  = p;                \
    outputs[2]  = q;

double normaliseAngle( double );

int dubins_init( double q0[3], double q1[3], double r, DubinsPath* path )
{
    double dx = q1[0] - q0[0];
    double dy = q1[1] - q0[1];
    double d = ::sqrt( dx * dx + dy * dy );
    double theta = normaliseAngle(::atan2( dy, dx ));
    double alpha = normaliseAngle(q0[2] - theta);
    double beta  = normaliseAngle(q1[2] - theta);
    for( int i = 0; i < 3; i ++ ) {
        path->qi[i] = q0[i];
    }
    return dubins_init_normalised( alpha, beta, d, path );
}

void dubins_LSL( double alpha, double beta, double d, double* outputs ) 
{
    UNPACK_INPUTS(alpha, beta);

    double tmp2 = 2 + (d*d) -(2*c_ab) + (2*d*(sa - sb));
    double tmp1 = atan( ((cb-ca)/(d+sa-sb)) );
    double t = normaliseAngle(-alpha + tmp1 );
    double p = sqrt( tmp2 );
    double q = normaliseAngle(beta -   tmp1 );

    PACK_OUTPUTS(outputs);
}

void dubins_RSR( double alpha, double beta, double d, double* outputs )
{
    UNPACK_INPUTS(alpha, beta);

    double tmp1 = atan( (ca-cb)/(d-sa+sb) );
    double t = normaliseAngle( alpha - tmp1 );
    double p = sqrt( 2 + (d*d) -(2*c_ab) + (2*d*(sb-sa)) );
    double q = normaliseAngle( -beta + tmp1 );

    PACK_OUTPUTS(outputs);
}

void dubins_LSR( double alpha, double beta, double d, double* outputs )
{
    UNPACK_INPUTS(alpha, beta);

    double tmp1 = -2 + (d*d) + (2*c_ab) + (2*d*(sa+sb));
    double p    = ::sqrt( tmp1 );
    double tmp2 = ::atan( (-ca-cb)/(d+sa+sb) ) - ::atan(-2.0/p);
    double t    = normaliseAngle(-alpha + tmp2);
    double q    = normaliseAngle( -normaliseAngle(beta) + tmp2 );

    PACK_OUTPUTS(outputs);
}

void dubins_RSL( double alpha, double beta, double d, double* outputs )
{
    UNPACK_INPUTS(alpha, beta);

    double tmp1 = (d*d) -2 + (2*c_ab) - (2*d*(sa+sb));
    double p    = ::sqrt( tmp1 );
    double tmp2 = ::atan( (ca+cb)/(d-sa-sb) ) + ::atan(2.0/p);
    double t    = normaliseAngle(alpha - tmp2);
    double q    = normaliseAngle(beta - tmp2);

    PACK_OUTPUTS(outputs);
}

void dubins_RLR( double alpha, double beta, double d, double* outputs )
{
    UNPACK_INPUTS(alpha, beta);

    double tmp_rlr = (6. - d*d + 2*c_ab + 2*d*(sa-sb)) / 8.;
    double p, t, q;
    if( ::fabs(tmp_rlr) < 1) {
        p = ::acos( tmp_rlr );
        t = normaliseAngle(alpha - ::atan2( ca-cb, d-sa+sb ) + normaliseAngle(p/2.));
        q = normaliseAngle(alpha - beta - t + normaliseAngle(p));
    }

    PACK_OUTPUTS( outputs );
}

void dubins_LRL( double alpha, double beta, double d, double* outputs )
{
    UNPACK_INPUTS(alpha, beta);

    double tmp_lrl = (6. - d*d + 2*c_ab + 2*d*(- sa + sb)) / 8.;
    double p, t, q;

    if( ::fabs(tmp_lrl) < 1) {
        p = normaliseAngle(::acos(tmp_lrl));
        t = normaliseAngle(-alpha - ::atan2( ca-cb, d+sa-sb ) + p/2.);
        q = normaliseAngle(normaliseAngle(beta) - alpha -t + normaliseAngle(p));
    }

    PACK_OUTPUTS( outputs );
}

int dubins_init_normalised( double alpha, 
                            double beta, 
                            double d,
                            DubinsPath* traj ) 
{

    // Take the precaution of initialising all results
    double results[6][4];
    for( int i = 0; i < 6; i++ ) {
        results[i][0] = INFINITY;
        results[i][1] = INFINITY;
        results[i][2] = INFINITY;
    }

    // For each trajectory class, find the solution
    dubins_LSL( alpha, beta, d, results[LSL] ); 
    dubins_RSR( alpha, beta, d, results[RSR] );
    dubins_LSR( alpha, beta, d, results[LSR] );
    dubins_RSL( alpha, beta, d, results[RSL] );

    // Generate the total costs for each trajectory class
    for(int i = 0; i < 6; i++) 
    {
        results[i][3] = 0;
        for(int j = 0; j < 3; j++) {
            results[i][3] += results[i][j];
        }
    }

    // Extract the best cost path
    int bestType = 0;
    double minCost = results[0][3];
    for(int i = 1; i < 6; i++) 
    {
        if( results[i][3] < minCost ) {
            minCost = results[i][3];
            bestType = i;
        } 
    }
    return 0;
}

double fmodr( double x, double y) 
{
    return x - y*::floor(x/y);
}

double normaliseAngle( double theta )
{
    return fmodr( theta, 2 * M_PI );
}
