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
    double alpha;
    double beta;
    double d;
    int ret;

    while(scanf("%lf %lf %lf", &alpha, &beta, &d) != EOF) {
        /* display input parameters */
        printf("# %f %f %f\n", alpha, beta, d);

        /* display information about the best path */
        DubinsPath path;
        double q0[] = {0.0, 0.0, alpha};
        double q1[] = {  d, 0.0, beta};
        int err = dubins_init(q0, q1, 1.0, &path);
        assert(err == 0);
        printf("# best = %d = %s\n", path.type, names[path.type]);

        /* display information about the specific words */
        int i;
        for(i = 0; i < 6; i++) {
            double params[3];
            params[0] = INFINITY;
            params[1] = INFINITY;
            params[2] = INFINITY;
            ret = dubins_words[i](alpha, beta, d, params);
            if(ret > 0) 
                ret = 1;
            double cost = params[0] + params[1] + params[2];
            printf("%s %.2f %d (%f %f %f)\n", names[i], cost, ret, params[0], params[1], params[2]);
        }

        /* blank line at the end of the block */
        printf("\n");
    }
    return 0;
}
