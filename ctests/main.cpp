#include "dubins.h"
#include <iostream>

int printConfiguration( double q[3], double x ) {
    std::cout << "configuration = ";
    for(int i = 0; i < 3; i++ ) {
        std::cout  << q[i] << " ";
    }
    std::cout << " at length = " << x << std::endl;
    return 0;
}

int main()
{
    double q0[] = { 0,0,0 };
    double q1[] = { 4,4,3.142 };
    DubinsPath path;
    dubins_init( q0, q1, 1.0, &path);
    dubins_path_sample_many( &path, printConfiguration, 0.1 );
    return 0;    
}
