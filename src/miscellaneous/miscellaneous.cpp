#include "miscellaneous.h"

/// Returns positive remainder
double p_remainder(double a, double b)
{
    double r = a - b * (int(a/b));
    if(r<0){ r += b; }
    return r;
}

// Returns -1, 0, 1 based on the input value
double p_sign(double input)
{
        if(input!=0.0)
        {
            return double((input > 0) - (input < 0));
        }
        else{ return 0.0; }
}