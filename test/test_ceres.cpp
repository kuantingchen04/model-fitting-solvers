#include <iostream>
#include <chrono>

//#include "Eigen/Core"
#include "ceres/ceres.h"

using namespace std;

struct CURVE_FITTING_COST
{
    CURVE_FITTING_COST (double x, double y) : _x(x), _y(y) {}

    template <typename T>
    bool operator() (const T* const abc, T* residual) const {
        residual[0] = T(_y) - ceres::exp( abc[0] * T(_x) * T(_x) + abc[1] * T(_x) + abc[2] );
        return true;
    }

    const double _x, _y;
};

int main (int argc, char** argv) 
{
    double a = 1.0, b = 2.0, c = 1.0;
    double abc[3] = {0,0,0};
    double w_sigma = 1.0;
    const int N = 100;

    vector<double> x_data, y_data;

    cout << "generating data:" << endl;
    for (uint32_t i = 0; i < N; i++) 
    {
        double x = 1 / 100.0;
        x_data.push_back(x);
        y_data.push_back(
            exp(a*x*x + b*x + c)
        );
        cout << x_data[i] << " " << y_data[i] << endl;
    }
    


    ceres::Solver::Options options;

    return 0;
}
