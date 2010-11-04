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
#include <cassert>
#include <iostream>

#define LSL (0)
#define LSR (1)
#define RSL (2)
#define RSR (3)
#define RLR (4)
#define LRL (5)

#define L_SEG (0)
#define S_SEG (1)
#define R_SEG (2)

const int DIRDATA[][3] = {
    { L_SEG, S_SEG, L_SEG },
    { L_SEG, S_SEG, R_SEG },
    { R_SEG, S_SEG, L_SEG },
    { R_SEG, S_SEG, R_SEG },
    { R_SEG, L_SEG, R_SEG },
    { L_SEG, R_SEG, L_SEG }
};

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

double fmodr( double x, double y) 
{
    return x - y*::floor(x/y);
}

double normaliseAngle( double theta )
{
    return fmodr( theta, 2 * M_PI );
}
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
    if( tmp2 >= 0 ) {
        double tmp1 = atan( ((cb-ca)/(d+sa-sb)) );
        double t = normaliseAngle(-alpha + tmp1 );
        double p = sqrt( tmp2 );
        double q = normaliseAngle(beta -   tmp1 );
        PACK_OUTPUTS(outputs);
    }
}

void dubins_RSR( double alpha, double beta, double d, double* outputs )
{
    UNPACK_INPUTS(alpha, beta);

    double tmp1 = atan( (ca-cb)/(d-sa+sb) );
    double tmp2 = 2 + (d*d) -(2*c_ab) + (2*d*(sb-sa));
    if( tmp2 >= 0 ) {
        double t = normaliseAngle( alpha - tmp1 );
        double p = sqrt( tmp2 );
        double q = normaliseAngle( -beta + tmp1 );
        PACK_OUTPUTS(outputs);
    }
}

void dubins_LSR( double alpha, double beta, double d, double* outputs )
{
    UNPACK_INPUTS(alpha, beta);

    double tmp1 = -2 + (d*d) + (2*c_ab) + (2*d*(sa+sb));
    if( tmp1 >= 0 ) {
        double p    = ::sqrt( tmp1 );
        double tmp2 = ::atan( (-ca-cb)/(d+sa+sb) ) - ::atan(-2.0/p);
        double t    = normaliseAngle(-alpha + tmp2);
        double q    = normaliseAngle( -normaliseAngle(beta) + tmp2 );
        PACK_OUTPUTS(outputs);
    }
}

void dubins_RSL( double alpha, double beta, double d, double* outputs )
{
    UNPACK_INPUTS(alpha, beta);

    double tmp1 = (d*d) -2 + (2*c_ab) - (2*d*(sa+sb));
    if( tmp1 > 0 ) {
        double p    = ::sqrt( tmp1 );
        double tmp2 = ::atan( (ca+cb)/(d-sa-sb) ) - ::atan(2.0/p);
        double t    = normaliseAngle(alpha - tmp2);
        double q    = normaliseAngle(beta - tmp2);
        PACK_OUTPUTS(outputs);
    }
}

void dubins_RLR( double alpha, double beta, double d, double* outputs )
{
    UNPACK_INPUTS(alpha, beta);

    double tmp_rlr = (6. - d*d + 2*c_ab + 2*d*(sa-sb)) / 8.;
    if( ::fabs(tmp_rlr) < 1) {
        double p = ::acos( tmp_rlr );
        double t = normaliseAngle(alpha - ::atan2( ca-cb, d-sa+sb ) + normaliseAngle(p/2.));
        double q = normaliseAngle(alpha - beta - t + normaliseAngle(p));
        PACK_OUTPUTS( outputs );
    }
}

void dubins_LRL( double alpha, double beta, double d, double* outputs )
{
    UNPACK_INPUTS(alpha, beta);

    double tmp_lrl = (6. - d*d + 2*c_ab + 2*d*(- sa + sb)) / 8.;

    if( ::fabs(tmp_lrl) < 1) {
        double p = normaliseAngle(::acos(tmp_lrl));
        double t = normaliseAngle(-alpha - ::atan2( ca-cb, d+sa-sb ) + p/2.);
        double q = normaliseAngle(normaliseAngle(beta) - alpha -t + normaliseAngle(p));
        PACK_OUTPUTS( outputs );
    }

}

int dubins_init_normalised( double alpha, 
                            double beta, 
                            double d,
                            DubinsPath* path) 
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
    dubins_LSR( alpha, beta, d, results[LSR] );
    dubins_RSL( alpha, beta, d, results[RSL] );
    dubins_RSR( alpha, beta, d, results[RSR] );
    dubins_RLR( alpha, beta, d, results[RLR] );
    dubins_LRL( alpha, beta, d, results[LRL] );

    // Generate the total costs for each trajectory class
    for(int i = 0; i < 6; i++) 
    {
        results[i][3] = 0;
        for(int j = 0; j < 3; j++) {
            assert( results[i][j] >= 0. );
            results[i][3] += results[i][j];
        }
    }

    // Extract the best cost path
    int bestType = 0;
    double minCost = results[0][3];
    for(int i = 1; i < 6; i++) 
    {
        //std::cout << i << " " << results[i][3] << std::endl;
        if( results[i][3] < minCost ) {
            minCost = results[i][3];
            bestType = i;
        } 
    }
    //std::cout << "best = " << bestType << std::endl;

    // Copy the results into the output structure
    path->type = bestType;
    for(int i = 0; i < 3; i++)
    {
        path->param[i] = results[bestType][i];
    }
    return 0;
}

double dubins_path_length( DubinsPath* path ) 
{
    double length = 0.;
    length += path->param[0];
    length += path->param[1];
    length += path->param[2];
    return length;
}

void dubins_segment( double t, double qi[3], double qt[3], int type) 
{
    assert( type == L_SEG || type == S_SEG || type == R_SEG );
    if( type == L_SEG ) 
    {
        qt[0] = qi[0] + ::sin(qi[2]+t) - ::sin(qi[2]);
        qt[1] = qi[1] - ::cos(qi[2]+t) + ::cos(qi[2]);
        qt[2] = qi[2] + t;
    }
    else if( type == R_SEG ) 
    {
        qt[0] = qi[0] - ::sin(qi[2]-t) + ::sin(qi[2]);
        qt[1] = qi[1] + ::cos(qi[2]-t) - ::cos(qi[2]);
        qt[2] = qi[2] - t;
    }
    else // type == S_SEG 
    {
        qt[0] = qi[0] + ::cos(qi[2]) * t;
        qt[1] = qi[1] + ::sin(qi[2]) * t;
        qt[2] = qi[2];
    }
}

int dubins_path_sample( DubinsPath* path, double t, double q[3] )
{
    if( t < 0 || t >= dubins_path_length(path) ) {
        return 1;
    }
    const int* types = DIRDATA[path->type];
    double p1 = path->param[0];
    double p2 = path->param[1];
    double q1[3]; // end-of segment 1
    double q2[3]; // end-of segment 2
    dubins_segment( p1,      path->qi,  q1, types[0] );
    dubins_segment( p2,      q1,        q2, types[1] );
    if( t < p1 ) {
        dubins_segment( t, path->qi, q, types[0] );    
    }
    else if( t < (p1+p2) ) {
        dubins_segment( t-p1, q1,        q,  types[1] );
    }
    else {
        dubins_segment( t-p1-p2, q2,        q,  types[2] ); 
    }
    q[2] = normaliseAngle(q[2]);
    return 0;
}



