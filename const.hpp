#ifndef CONST_HPP
#define CONST_HPP

#include <vector>
#include <complex>
#include <numeric>
#include <algorithm>

namespace CV{
    double PI = 3.1415926535;
    uint16_t CW = 800;
    uint16_t CH = 800;
    
    template<class T>
        std::vector<double> G(){
            return {0,0,-1};
        }
};

#endif
