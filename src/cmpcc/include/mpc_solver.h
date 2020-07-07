#ifndef PROJECT_MPC_SOLVER_H
#define PROJECT_MPC_SOLVER_H
#include "model.h"
#include "map.h"
#include "osqp_interface.h"
#include "display_msgs.h"

namespace ft{
    class MpcSolver
    {
    private:
        int corridorConstrains; // set to 1 if considering corridor constrains
        double qVTheta; //cost for v_theta : set in config/qVTheta.yaml
        std::vector<double> stateUpper;
        std::vector<double> stateLower;
        std::vector<double> inputUpper;
        std::vector<double> inputLower;
        Eigen::SparseMatrix<double> Inu;
        int numState = Model::numState; // ns
        int numInput = Model::numInput; // ni
        /**
         * Q = BB'(Qk)BB
         * c = BB'(Qk'AA x0 + qk)
         * X = BB * U + AA * x0
         * **/
        // matrices for qpsolver: 
        Eigen::SparseMatrix<double> Q ; // N*ns * N*ns
        Eigen::SparseMatrix<double> c ; // N*ni * 1
        Eigen::SparseMatrix<double> A ;
        Eigen::SparseMatrix<double> b ;
        Eigen::SparseMatrix<double> C ;
        Eigen::SparseMatrix<double> clow;
        Eigen::SparseMatrix<double> cupp;
        // xlow : ul; xupp : uu;
        // const matrices: 
        Eigen::SparseMatrix<double> AA; // N*ns * ns
        Eigen::SparseMatrix<double> BB; // N*ns * N*ni
        Eigen::SparseMatrix<double> AAT;//AA.transpose()
        Eigen::SparseMatrix<double> BBT;//BB.transpose()
        Eigen::SparseMatrix<double> In; // ns   *   ns : tiny diag
        Eigen::SparseMatrix<double> xu; // N*ns *    1 : state upper
        Eigen::SparseMatrix<double> xl; // N*ns *    1 : state lower
        Eigen::SparseMatrix<double> uu; // N*ns *    1 : input upper
        Eigen::SparseMatrix<double> ul; // N*ns *    1 : input lower
        // temp matrices for each horizon: 
        Eigen::SparseMatrix<double> Qn; // ns   *   ns
        Eigen::SparseMatrix<double> qn; // ns   *    1
        Eigen::SparseMatrix<double> Cn; // n?   *   ns
        Eigen::SparseMatrix<double> Cnb;// n?   *    1 
        Eigen::SparseMatrix<double> Qk; // N*ns * N*ns : blkdiag(Q1,Q2,Q3,...)
        Eigen::SparseMatrix<double> qk; // N*ns *    1 : [q1;q2;q3;...]
        Eigen::SparseMatrix<double> Ck; // n??  *   ns : blkdiag(C1,C2,C3,...)
        Eigen::SparseMatrix<double> Ckb;// n??  *    1

        osqp::OSQPInterface osqpInterface; // qpSolver interface

    public:
        bool initStatus;
        int horizon; // N : set in config/qVTheta.yaml
        ft::Map map;
        ft::Model model;
        // solution: 
        Eigen::SparseMatrix<double> inputPredict;
        Eigen::SparseMatrix<double> statePredict;
        DisplayMsgs * displayPtr;

        MpcSolver();
        ~MpcSolver();
        void calculateCost(double theta, int horizon_);
        void calculateConstrains(Eigen::SparseMatrix<double> &stateTmp, int horizon_);
        int solveMpcQp(Eigen::SparseMatrix<double> &stateTmp);
        void printMatrices(); // just for debug
    };
}//namespace ft

#endif