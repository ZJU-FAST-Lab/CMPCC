#include "mpc_solver.h"

using namespace Eigen;
using namespace std;

int main(int argc, char const *argv[])
{
    using namespace ft;
    ft::Map map;
    MpcSolver solver;
    
    SparseMatrix<double> state(Model::numState,1);
    double theta = 0.01;
    Eigen::Vector3d pos, vel;
    map.getGlobalCommand(theta, pos, vel);
    cout << "theta_max: " << map.thetaMax << endl;
    cout << "pos_test: " << pos.transpose() << endl;
    cout << "vel_tess: " << vel.transpose() << endl;
    state.coeffRef(0,0) = pos(0);
    state.coeffRef(Model::numOrder,0) = pos(1);
    state.coeffRef(2*Model::numOrder,0) = pos(2);
    state.coeffRef(Model::numState-Model::numOrder,0) = theta;
    int solveStatus = solver.solveMpcQp(state);
    cout << solveStatus <<endl;
    cout << solver.inputPredict << endl;
}
