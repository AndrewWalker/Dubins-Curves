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
#include <algorithm> 

using namespace std; 

#define EPSILON (10e-10)

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

#define UNPACK_INPUTS(alpha, beta)     \
    double sa = sin(alpha);            \
    double sb = sin(beta);             \
    double ca = cos(alpha);            \
    double cb = cos(beta);             \
    double c_ab = cos(alpha - beta);   \
    
#define PACK_OUTPUTS(outputs)       \
    outputs[0]  = t;                \
    outputs[1]  = p;                \
    outputs[2]  = q;

/**
 * Floating point modulus suitable for rings
 * 
 * fmod doesn't behave correctly for angular quantities, this function does
 */
double fmodr( double x, double y) 
{
    return x - y*floor(x/y);
}

double mod2pi( double theta )
{
    return fmodr( theta, 2 * M_PI );
}

int dubins_init( double q0[3], double q1[3], double rho, DubinsPath* path )
{
    double dx = q1[0] - q0[0];
    double dy = q1[1] - q0[1];
    double D = sqrt( dx * dx + dy * dy );
    double d = D / rho;
    if( rho <= 0. ) {
        return EDUBBADRHO;
    }
    if( (fabs(dx) < EPSILON) && (fabs(dy) < EPSILON) ) {
        // if the configurations are colocated, theta has no clear definition
        // TODO test if you can get away with letting theta = 0
        return EDUBCOCONFIGS;
    }
    double theta = mod2pi(atan2( dy, dx ));
    double alpha = mod2pi(q0[2] - theta);
    double beta  = mod2pi(q1[2] - theta);
    for( int i = 0; i < 3; i ++ ) {
        path->qi[i] = q0[i];
    }
    return dubins_init_normalised( alpha, beta, d, rho, path );
}

void dubins_LSL( double alpha, double beta, double d, double* outputs ) 
{
    UNPACK_INPUTS(alpha, beta);

    double tmp0 = d+sa-sb;
    double tmp2 = 2 + (d*d) -(2*c_ab) + (2*d*(sa - sb));
    if( tmp2 >= 0 && tmp0 >= 0) {
        double tmp1 = atan( (cb-ca)/tmp0 );
        double t = mod2pi(-alpha + tmp1 );
        double p = sqrt( tmp2 );
        double q = mod2pi(beta -   tmp1 );
        PACK_OUTPUTS(outputs);
    }
}

void dubins_RSR( double alpha, double beta, double d, double* outputs )
{
    UNPACK_INPUTS(alpha, beta);

    double tmp0 = d-sa+sb;
    double tmp2 = 2 + (d*d) -(2*c_ab) + (2*d*(sb-sa));
    if( tmp2 >= 0 && tmp0 >= 0) {
        double tmp1 = atan( (ca-cb)/tmp0 );
        double t = mod2pi( alpha - tmp1 );
        double p = sqrt( tmp2 );
        double q = mod2pi( -beta + tmp1 );
        PACK_OUTPUTS(outputs);
    }
}

void dubins_LSR( double alpha, double beta, double d, double* outputs )
{
    UNPACK_INPUTS(alpha, beta);

    double tmp1 = -2 + (d*d) + (2*c_ab) + (2*d*(sa+sb));
    if( tmp1 >= 0 ) {
        double p    = sqrt( tmp1 );
        double tmp2 = atan( (-ca-cb)/(d+sa+sb) ) - atan(-2.0/p);
        double t    = mod2pi(-alpha + tmp2);
        double q    = mod2pi( -mod2pi(beta) + tmp2 );
        PACK_OUTPUTS(outputs);
    }
}

void dubins_RSL( double alpha, double beta, double d, double* outputs )
{
    UNPACK_INPUTS(alpha, beta);

    double tmp1 = (d*d) -2 + (2*c_ab) - (2*d*(sa+sb));
    if( tmp1 > 0 ) {
        double p    = sqrt( tmp1 );
        double tmp2 = atan( (ca+cb)/(d-sa-sb) ) - atan(2.0/p);
        double t    = mod2pi(alpha - tmp2);
        double q    = mod2pi(beta - tmp2);
        PACK_OUTPUTS(outputs);
    }
}

void dubins_RLR( double alpha, double beta, double d, double* outputs )
{
    UNPACK_INPUTS(alpha, beta);

    double tmp_rlr = (6. - d*d + 2*c_ab + 2*d*(sa-sb)) / 8.;
    if( fabs(tmp_rlr) < 1) {
        double p = acos( tmp_rlr );
        double t = mod2pi(alpha - atan2( ca-cb, d-sa+sb ) + mod2pi(p/2.));
        double q = mod2pi(alpha - beta - t + mod2pi(p));
        PACK_OUTPUTS( outputs );
    }
}

void dubins_LRL( double alpha, double beta, double d, double* outputs )
{
    UNPACK_INPUTS(alpha, beta);

    double tmp_lrl = (6. - d*d + 2*c_ab + 2*d*(- sa + sb)) / 8.;

    if( fabs(tmp_lrl) < 1) {
        double p = mod2pi(acos(tmp_lrl));
        double t = mod2pi(-alpha - atan2( ca-cb, d+sa-sb ) + p/2.);
        double q = mod2pi(mod2pi(beta) - alpha -t + mod2pi(p));
        PACK_OUTPUTS( outputs );
    }

}

int dubins_init_normalised( double alpha, 
                            double beta, 
                            double d,
                            double rho,
                            DubinsPath* path) 
{
    path->rho = rho;

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
            //assert( results[i][j] >= 0. );
            //std::cout << results[i][j] << " ";
            results[i][3] += results[i][j];
        }
        //std::cout << std::endl;
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
    length = length * path->rho;
    return length;
}

void dubins_segment( double t, double qi[3], double qt[3], int type) 
{
    assert( type == L_SEG || type == S_SEG || type == R_SEG );
    if( type == L_SEG ) 
    {
        qt[0] = qi[0] + sin(qi[2]+t) - sin(qi[2]);
        qt[1] = qi[1] - cos(qi[2]+t) + cos(qi[2]);
        qt[2] = qi[2] + t;
    }
    else if( type == R_SEG ) 
    {
        qt[0] = qi[0] - sin(qi[2]-t) + sin(qi[2]);
        qt[1] = qi[1] + cos(qi[2]-t) - cos(qi[2]);
        qt[2] = qi[2] - t;
    }
    else // type == S_SEG 
    {
        qt[0] = qi[0] + cos(qi[2]) * t;
        qt[1] = qi[1] + sin(qi[2]) * t;
        qt[2] = qi[2];
    }
}

int dubins_path_sample( DubinsPath* path, double t, double q[3] )
{
    if( t < 0 || t >= dubins_path_length(path) ) {
        // error, parameter out of bounds
        return EDUBPARAM;
    }

    // tprime is the normalised variant of the parameter t
    double tprime = t / path->rho;

    // In order to take rho != 1 into account this function needs to be more complex
    // than it would be otherwise. The transformation is done in five stages.
    //
    // 1. translate the components of the initial configuration to the origin
    // 2. generate the target configuration
    // 3. transform the target configuration
    //      scale the target configuration
    //      translate the target configration back to the original starting point
    //      normalise the target configurations angular component
    
    // The translated initial configuration 
    double qi[3] = { 0, 0, path->qi[2] };

    // Generate the target configuration
    const int* types = DIRDATA[path->type];
    double p1 = path->param[0];
    double p2 = path->param[1];
    double q1[3]; // end-of segment 1
    double q2[3]; // end-of segment 2
    dubins_segment( p1,      qi,    q1, types[0] );
    dubins_segment( p2,      q1,    q2, types[1] );
    if( tprime < p1 ) {
        dubins_segment( tprime, qi, q, types[0] );    
    }
    else if( tprime < (p1+p2) ) {
        dubins_segment( tprime-p1, q1, q,  types[1] );
    }
    else {
        dubins_segment( tprime-p1-p2, q2, q,  types[2] ); 
    }

    // scale the target configuration, translate back to the original starting point
    q[0] = q[0] * path->rho + path->qi[0];
    q[1] = q[1] * path->rho + path->qi[1];
    q[2] = mod2pi(q[2]);

    return 0;
}

int dubins_path_sample_many( DubinsPath* path, DubinsPathSamplingCallback cb, double stepSize ) 
{
    // TODO - this implementation could be optimised by caching
    //        some of the calculations from dubins_path_sample

    double x = 0.0;
    double length = dubins_path_length(path);
    while( x <  length ) {
        double q[3];
        dubins_path_sample( path, x, q );
        int retcode = cb(q, x);
        if( retcode != 0 ) {
            return retcode;    
        }
        x += stepSize;
    } 
    return 0;
}

int dubins_path_endpoint( DubinsPath* path, double q[3] )
{
    // TODO - introduce a new constant rather than just using EPSILON
    return dubins_path_sample( path, dubins_path_length(path) - EPSILON, q );
}

int dubins_extract_subpath( DubinsPath* path, double t, DubinsPath* newpath )
{
    // calculate the true parameter
    double tprime = t / path->rho;

    // copy most of the data
    newpath->qi[0] = path->qi[0];
    newpath->qi[1] = path->qi[1];
    newpath->qi[2] = path->qi[2];
    newpath->rho   = path->rho;
    newpath->type  = path->type;

    // fix the parameters
    newpath->param[0] = std::min( path->param[0], tprime );
    newpath->param[1] = std::min( path->param[1], tprime - newpath->param[0]);
    newpath->param[2] = std::min( path->param[2], tprime - newpath->param[0] - newpath->param[1]);
    return 0;    
}


