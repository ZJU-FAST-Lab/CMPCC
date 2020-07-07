#include <Eigen/Sparse>
#include <iostream>
#include "osqp_interface.h"

using namespace Eigen;
using namespace std;
using namespace osqp;

void printQ(Eigen::SparseMatrix<double> Q_){
  cout << Q_ << endl;
  for(int i=0; i<Q_.nonZeros(); ++i){
    cout << Q_.innerIndexPtr()[i] << " ";
  }
  cout << endl;
  for(int i=0; i<Q_.cols()+1; ++i){
    cout << Q_.outerIndexPtr()[i] << " ";
  }
  cout << endl;
}

int main(int argc, char const *argv[])
{
  Eigen::SparseMatrix<double> Q_; 
  Eigen::SparseMatrix<double> c_; 
  Eigen::SparseMatrix<double> A_; 
  Eigen::SparseMatrix<double> b_; 
  Eigen::SparseMatrix<double> C_; 
  Eigen::SparseMatrix<double> clow_;
  Eigen::SparseMatrix<double> cupp_;
  Eigen::SparseMatrix<double> xlow_;
  Eigen::SparseMatrix<double> xupp_;
  Q_.resize(2,2);
  Q_.coeffRef(0,0) = 4;
  Q_.coeffRef(0,1) = 1;
  Q_.coeffRef(1,1) = 2;
  c_.resize(2,1);
  c_.coeffRef(0,0) = 1;
  c_.coeffRef(1,0) = 1;
  C_.resize(3,2);
  C_.coeffRef(0,0) = 1;
  C_.coeffRef(1,0) = 1;
  C_.coeffRef(0,1) = 1;
  C_.coeffRef(2,1) = 1;
  clow_.resize(3,1);
  clow_.coeffRef(0,0) = 1;
  clow_.coeffRef(1,0) = 0;
  clow_.coeffRef(2,0) = 0;
  cupp_.resize(3,1);
  cupp_.coeffRef(0,0) = 1;
  cupp_.coeffRef(1,0) = 0.7;
  cupp_.coeffRef(2,0) = 0.7;
  // printQ(Q_);
  osqp::OSQPInterface osqpInterface;
  osqpInterface.updateMatrices(Q_, c_, A_, b_, C_, clow_, cupp_, xlow_, xupp_);
  osqpInterface.solveQP();
  auto solPtr = osqpInterface.solPtr();
  cout << solPtr->x[0] << endl;
  return 0;
}
