#include <iostream>
#include <random>
#include <chrono>

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
    std::random_device rd{};
    std::mt19937 gen{rd()};
    normal_distribution<> dist{0, w_sigma};

    // generate data
    for (uint32_t i = 0; i < N; i++) 
    {
        const double x = i / 100.0;
        x_data.push_back(x);
        y_data.push_back( exp(a*x*x + b*x + c) + dist(gen) );
    }
    
    // least square problem
    ceres::Problem problem;
    for (uint32_t i = 0; i < N; i++)
    {
        problem.AddResidualBlock(
            new ceres::AutoDiffCostFunction<CURVE_FITTING_COST, 1, 3>(
                new CURVE_FITTING_COST (x_data[i], y_data[i])),
            nullptr,
            abc
        );
    }

    // solver
    ceres::Solver::Options options;
    options.linear_solver_type = ceres::DENSE_QR;
    options.minimizer_progress_to_stdout = true;

    ceres::Solver::Summary summary;
    chrono::steady_clock::time_point t1 = chrono::steady_clock::now();
    ceres::Solve(options, &problem, &summary);
    chrono::steady_clock::time_point t2 = chrono::steady_clock::now();
    chrono::duration<double> tt = chrono::duration_cast<chrono::duration<double>>(t2 - t1);
    cout << "solver time:" << tt.count() << endl;

    cout << summary.BriefReport() << endl;
    cout << "estimated a,b,c = ";
    for (auto x:abc)
        cout << x << " ";
    cout << endl;
    return 0;
}
