#include "dubins.h"
#include <iostream>

int main()
{
    double q0[] = { 0,0,0 };
    double q1[] = { 4,4,3.142 };
    DubinsPath path;
    dubins_init( q0, q1, 1.0, &path);
    double length = dubins_path_length( &path );
    double dx = 0.1;
    double x = 0.0;
    double q[3];
    while( x < length ) {
        dubins_path_sample( &path, x, q );
        std::cout << "configuration = ";
        for(int i = 0; i < 3; i++ ) {
            std::cout  << q[i] << " ";
        }
        std::cout << " at length = " << x << std::endl;
        x += dx;
    }
    return 0;    
}
