#include "dubins.h"
#include <stdio.h>

int printConfiguration( double q[3], double x ) {
    printf("%f,%f,%f,%f\n", q[0], q[1], q[2], x);
    return 0;
}

int main()
{
    double q0[] = { 0,0,0 };
    double q1[] = { 4,4,3.142 };
    DubinsPath path;
    dubins_init( q0, q1, 1.0, &path);

    printf("#x,y,theta,t\n");
    dubins_path_sample_many( &path, printConfiguration, 0.1 );

    return 0;
}
