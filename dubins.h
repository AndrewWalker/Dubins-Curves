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
#ifndef DUBINS_H
#define DUBINS_H

extern "C" {

struct DubinsPath
{
    double qi[3];
    double param[3];
    int type;
};


/**
 * Generate a trajectory from an initial configuration to
 * a target configuration, with a specified maximum turning
 * radii
 *
 * @param q0    - a configuration specified as an array of x, y, theta
 * @param q1    - a configuration specified as an array of x, y, theta
 * @param r     - the turning radius in the same units as x and y
 * @param traj  - the resultant trajectory
 * @return      - 0 on success
 */
int dubins_init( double q0[3], double q1[3], double r, DubinsPath* traj );

int dubins_init_normalised( double alpha, double beta, double d, DubinsPath* path );

void dubins_LSL( double alpha, double beta, double d, double* outputs );
void dubins_RSR( double alpha, double beta, double d, double* outputs );
void dubins_LSR( double alpha, double beta, double d, double* outputs );
void dubins_RSL( double alpha, double beta, double d, double* outputs );
void dubins_LRL( double alpha, double beta, double d, double* outputs );
void dubins_RLR( double alpha, double beta, double d, double* outputs );

} // extern "C"

#endif // DUBINS_H


