#include "osqp_interface.h"
#include <iostream>

using namespace std;
using namespace Eigen;

namespace osqp{
OSQPInterface::OSQPInterface(){
    settings = new OSQPSettings;
    data     = new OSQPData;
}
OSQPInterface::~OSQPInterface(){
    delete settings;
    delete data;
    osqp_cleanup(work);
}
int OSQPInterface::updateMatrices(
    Eigen::SparseMatrix<double> & Q_, 
    Eigen::SparseMatrix<double> & c_, 
    Eigen::SparseMatrix<double> & A_, 
    Eigen::SparseMatrix<double> & b_, 
    Eigen::SparseMatrix<double> & C_, 
    Eigen::SparseMatrix<double> & clow_, 
    Eigen::SparseMatrix<double> & cupp_,
    Eigen::SparseMatrix<double> & xlow_,
    Eigen::SparseMatrix<double> & xupp_,
    int warmStart
){
    // change Q_ upper triangular
    Q_ = Q_.triangularView<Upper>();
    // compress the matrices for osqp data
    Q_.makeCompressed();
    c_.makeCompressed();
    A_.makeCompressed();
    b_.makeCompressed();
    C_.makeCompressed();
    clow_.makeCompressed();
    cupp_.makeCompressed();
    xlow_.makeCompressed();
    xupp_.makeCompressed();

    data->n = Q_.rows();
    data->m = C_.rows();
    // use the eigen sparse ptrs make it faster
    data->P = csc_matrix(data->n, data->n, Q_.nonZeros(), 
        Q_.valuePtr(), Q_.innerIndexPtr(), Q_.outerIndexPtr());
    data->q = new c_float[c_.rows()];
    for (int i=0; i<c_.rows(); ++i){
        data->q[i] = c_.coeffRef(i,0);
    }
    data->A = csc_matrix(data->m, data->n, C_.nonZeros(), 
        C_.valuePtr(), C_.innerIndexPtr(), C_.outerIndexPtr());
    data->l = new c_float[clow_.rows()];
    for (int i=0; i<clow_.rows(); ++i){
        data->l[i] = clow_.coeffRef(i,0);
    }
    data->u = new c_float[cupp_.rows()];
    for (int i=0; i<cupp_.rows(); ++i){
        data->u[i] = cupp_.coeffRef(i,0);
    }
    if (settings) osqp_set_default_settings(settings);
    settings->warm_start = warmStart;
    exitflag = osqp_setup(&work, data, settings);
    delete data->q;
    delete data->l;
    delete data->u;
    return 0;
}
int OSQPInterface::solveQP(){
    // Solve Problem
    return osqp_solve(work);
}
int OSQPInterface::solveStatus(){
    return work->info->status_val;
}
OSQPSolution* OSQPInterface::solPtr(){
    return work->solution;
}
}// namespace osqp