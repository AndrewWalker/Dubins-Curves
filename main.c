#include "dubins.h"

int main()
{
    double q0[] = { 0,0,0 };
    double q1[] = { 4,4,3.142 };
    DubinsPath path;
    dubins_init( q0, q1, 1.0, &path);
    return 0;    
}
