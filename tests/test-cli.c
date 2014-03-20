// Copyright (c) 2008-2014, Andrew Walker
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
#include <stdio.h>
#include <math.h>
#include <assert.h>

char* names[] = {
    "LSL",
    "LSR",
    "RSL",
    "RSR",
    "RLR",
    "LRL",
};

int main()
{
    double q0[3];
    double q1[3];
    double turning_radius;

    while(scanf("%lf %lf %lf %lf %lf %lf %lf", 
                &q0[0], &q0[1], &q0[2], 
                &q1[0], &q1[1], &q1[2], 
                &turning_radius) != EOF) 
    {
        /* display information about the best path */
        DubinsPath path;
        int err = dubins_init(q0, q1, turning_radius, &path);
        assert(err == 0);
        printf("# best = %d = %s\n", path.type, names[path.type]);
    }
    return 0;
}
