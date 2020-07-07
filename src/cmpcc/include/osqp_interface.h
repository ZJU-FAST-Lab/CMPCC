#ifndef PROJECT_OSQP_INTERFACE_H
#define PROJECT_OSQP_INTERFACE_H
#include <osqp/osqp.h>
#include <Eigen/Sparse>

namespace osqp{
class OSQPInterface{
private:
    c_int   P_nnz;
    c_int   A_nnz;
    // Exitflag
    c_int exitflag = 0;

    // Workspace structures
    OSQPWorkspace *work;
    OSQPSettings  *settings;
    OSQPData      *data;

public:
    OSQPInterface();
    ~OSQPInterface();
    int updateMatrices(
        Eigen::SparseMatrix<double> & Q_, 
        Eigen::SparseMatrix<double> & c_, 
        Eigen::SparseMatrix<double> & A_, 
        Eigen::SparseMatrix<double> & b_, 
        Eigen::SparseMatrix<double> & C_, 
        Eigen::SparseMatrix<double> & clow_, 
        Eigen::SparseMatrix<double> & cupp_,
        Eigen::SparseMatrix<double> & xlow_,
        Eigen::SparseMatrix<double> & xupp_,
        int warmStart = WARM_START
    );
    int solveQP();
    int solveStatus();
    OSQPSolution* solPtr();
};
} // namespace osqp


#endif


