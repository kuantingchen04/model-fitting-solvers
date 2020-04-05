#include <iostream>
#include <random>
#include <chrono>

#include <Eigen/Core>
#include <g2o/core/base_vertex.h>
#include <g2o/core/base_unary_edge.h>
#include <g2o/core/block_solver.h>
#include <g2o/solvers/dense/linear_solver_dense.h>
#include <g2o/core/optimization_algorithm_levenberg.h>
#include <g2o/core/optimization_algorithm_gauss_newton.h>
#include <g2o/core/optimization_algorithm_dogleg.h>

using namespace std;

// vertex: the params, a, b, c
class VertexParams: public g2o::BaseVertex<3, Eigen::Vector3d>
{
public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW;
    VertexParams()
    {
    }
    virtual bool read(std::istream& /*is*/)
    {
        cerr << __PRETTY_FUNCTION__ << " not implemented yet" << endl;
        return false;
    }

    virtual bool write(std::ostream& /*os*/) const
    {
        cerr << __PRETTY_FUNCTION__ << " not implemented yet" << endl;
        return false;
    }
    virtual void setToOriginImpl()
    {
        cerr << __PRETTY_FUNCTION__ << " not implemented yet" << endl;
    }
    virtual void oplusImpl(const double* update)
    {
        Eigen::Vector3d::ConstMapType v(update);
        _estimate += v;
    }
};

// edge: error model
class EdgePointOnCurve : public g2o::BaseUnaryEdge<1, Eigen::Vector2d, VertexParams>
{
public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    EdgePointOnCurve()
    {
    }
    virtual bool read(std::istream& /*is*/)
    {
        cerr << __PRETTY_FUNCTION__ << " not implemented yet" << endl;
        return false;
    }
    virtual bool write(std::ostream& /*os*/) const
    {
        cerr << __PRETTY_FUNCTION__ << " not implemented yet" << endl;
        return false;
    }
    void computeError()
    {
        const VertexParams* params = static_cast<const VertexParams*>(vertex(0));
        const double& a = params->estimate()(0);
        const double& b = params->estimate()(1);
        const double& c = params->estimate()(2);
        double fval = exp(a*measurement()(0)*measurement()(0) + b*measurement()(0) + c);
        _error(0) = fval - measurement()(1);
    }
};

int main (int argc, char** argv)
{
    double a = 1.0, b = 2.0, c = 1.0;
    double abc[3] = {0,0,0};
    double w_sigma = 1.0;
    const int N = 100;

    vector<Eigen::Vector2d> points (N);
    std::random_device rd{};
    std::mt19937 gen{rd()};
    normal_distribution<> dist{0, w_sigma};

    // generate data
    for (uint32_t i = 0; i < N; i++)
    {
        const double x = i / 100.0;
        points[i].x() = x;
        points[i].y() = exp(a*x*x + b*x + c) + dist(gen);
    }

    // some handy typedefs
    typedef g2o::BlockSolver<g2o::BlockSolverTraits<3,1>> MyBlockSolver;
    typedef g2o::LinearSolverDense<MyBlockSolver::PoseMatrixType> MyLinearSolver;

    // setup the solver
    g2o::SparseOptimizer optimizer;
    optimizer.setVerbose(true);

    g2o::OptimizationAlgorithmLevenberg* solver = new g2o::OptimizationAlgorithmLevenberg(
            g2o::make_unique<MyBlockSolver>(g2o::make_unique<MyLinearSolver>()));

//    g2o::OptimizationAlgorithmLevenberg* solver = new g2o::OptimizationAlgorithmGaussNewton(
//            g2o::make_unique<MyBlockSolver>(g2o::make_unique<MyLinearSolver>()));

//    g2o::OptimizationAlgorithmLevenberg* solver = new g2o::OptimizationAlgorithmDogleg(
//            g2o::make_unique<MyBlockSolver>(g2o::make_unique<MyLinearSolver>()));

    optimizer.setAlgorithm(solver);

    // build the optimization problem given the points
    // 1. add the parameter vertex
    VertexParams* v = new VertexParams();
    v->setId(0);
    v->setEstimate(Eigen::Vector3d(0,0,0)); // some initial value for the params
    optimizer.addVertex(v);
    // 2. add the points we measured to be on the curve
    for (int i = 0; i < N; i++)
    {
        EdgePointOnCurve* e = new EdgePointOnCurve;
        e->setInformation(Eigen::Matrix<double, 1, 1>::Identity());
        e->setId(i);
        e->setVertex(0, v);
        e->setMeasurement(points[i]);
        optimizer.addEdge(e);
    }

    // perform the optimization
    chrono::steady_clock::time_point t1 = chrono::steady_clock::now();
    optimizer.initializeOptimization();
    optimizer.setVerbose(true);
    optimizer.optimize(100);
    chrono::steady_clock::time_point t2 = chrono::steady_clock::now();
    chrono::duration<double> tt = chrono::duration_cast<chrono::duration<double>>(t2 - t1);
    cout << "solver time:" << tt.count() << endl;

    // print out the result
    cout << "estimated a,b,c = "
            << v->estimate()(0) << " "
            << v->estimate()(1) << " "
            << v->estimate()(2) << endl;
    return 0;
}