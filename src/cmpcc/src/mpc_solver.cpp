#include "mpc_solver.h"
#include "sparse_utils.h"
#include <iostream>
#include <ros/package.h>
#include <ctime>

using namespace Eigen;
using namespace std;
namespace ft{
    MpcSolver::MpcSolver()
    {
        std::string path = ros::package::getPath("cmpcc")+"/config/mpcParameters.yaml";
        YAML::Node node = YAML::LoadFile(path);
        horizon = node["horizon"].as<int>();
        corridorConstrains = node["corridorConstrains"].as<int>();
        qVTheta = node["qVTheta"].as<double>();
        stateUpper = node["stateUpper"].as<vector<double>>();
        stateLower = node["stateLower"].as<vector<double>>();
        inputUpper = node["inputUpper"].as<vector<double>>();
        inputLower = node["inputLower"].as<vector<double>>();
        displayPtr = new DisplayMsgs(map, horizon);
        initStatus = true;

        Qk.resize(horizon*numState, horizon*numState);
        qk.resize(horizon*numState, 1);
        AA.resize(horizon*numState, numState);
        BB.resize(horizon*numState, horizon*numInput);
        In.resize(horizon*numInput, horizon*numInput);
        A.resize(0,0);
        b.resize(0,0);
        xl.resize(horizon*numState, 1);
        xu.resize(horizon*numState, 1);
        ul.resize(horizon*numInput, 1);
        uu.resize(horizon*numInput, 1);
        inputPredict.resize(numInput, horizon);
        statePredict.resize(numState, horizon);
        // initialize xl,xu,ul,uu: 
        Eigen::SparseMatrix<double> stateUs(numState,1);
        Eigen::SparseMatrix<double> stateLs(numState,1);
        Eigen::SparseMatrix<double> inputUs(numInput,1);
        Eigen::SparseMatrix<double> inputLs(numInput,1);
        for (int i=0; i<numState; ++i){
            stateUs.coeffRef(i,0) = stateUpper[i];
            stateLs.coeffRef(i,0) = stateLower[i];
        }
        for (int i=0; i<numInput; ++i){
            inputUs.coeffRef(i,0) = inputUpper[i];
            inputLs.coeffRef(i,0) = inputLower[i];
        }
        for (int i=0; i<horizon; ++i){
            sp::colMajor::setRows(xl, stateLs, numState*i);
            sp::colMajor::setRows(xu, stateUs, numState*i);
            sp::colMajor::setRows(ul, inputLs, numInput*i);
            sp::colMajor::setRows(uu, inputUs, numInput*i);
            xu.coeffRef(i*numState+9,0) = map.thetaMax;
        }
        // initialize AA & BB: 
        Eigen::SparseMatrix<double> tmpA = model.Ad;
        Eigen::SparseMatrix<double> tmpB(numState, horizon*numInput);
        sp::colMajor::setRows(AA, model.Ad, 0);
        sp::colMajor::setBlock(BB, model.Bd, 0, 0);
        for (int k=1; k<horizon; ++k){
            tmpA = model.Ad * tmpA;
            sp::colMajor::setRows(AA, tmpA, k*numState);
            tmpB = model.Ad * BB.block((k-1)*numState,0,numState,horizon*numInput);
            sp::colMajor::setRows(BB, tmpB, k*numState);
            sp::colMajor::setBlock(BB, model.Bd, k*numState, k*numInput);
        }
        AAT = AA.transpose();
        BBT = BB.transpose();
        // initialize In: -> penalty minium snap
        for (int i=0; i<horizon*numInput; ++i){
            In.coeffRef(i,i) = 1e-4;
            if (i < (horizon-1)*numInput){
                In.coeffRef(i,i+numInput) = -1e-6;
                In.coeffRef(i+numInput,i) = -1e-6;
            }
        }
        // initialize Inu:
        Inu.resize(horizon*numInput, horizon*numInput);
        Inu.setIdentity();
    }

    MpcSolver::~MpcSolver(){
        delete displayPtr;
    }

    void MpcSolver::calculateCost(double theta, int horizon_){
        double theta_ = std::fmod(theta, map.thetaMax);
        Eigen::Vector3d pos, vel;
        map.getGlobalCommand(theta_, pos, vel);
        double x_v = pos(0);
        double y_v = pos(1);
        double z_v = pos(2);
        double dx_v__dtheta = vel(0);
        double dy_v__dtheta = vel(1);
        double dz_v__dtheta = vel(2);
        double r_x = x_v - dx_v__dtheta * theta;
        double r_y = y_v - dy_v__dtheta * theta;
        double r_z = z_v - dz_v__dtheta * theta;
        Eigen::SparseMatrix<double> grad_x(numState, 1);
        Eigen::SparseMatrix<double> grad_y(numState, 1);
        Eigen::SparseMatrix<double> grad_z(numState, 1);
        grad_x.coeffRef(0,0) = 1;
        grad_y.coeffRef(Model::numOrder,0) = 1;
        grad_z.coeffRef(2*Model::numOrder,0) = 1;
        grad_x.coeffRef(numState-Model::numOrder,0) = -dx_v__dtheta;
        grad_y.coeffRef(numState-Model::numOrder,0) = -dy_v__dtheta;
        grad_z.coeffRef(numState-Model::numOrder,0) = -dz_v__dtheta;
        Qn = grad_x * Eigen::SparseMatrix<double>(grad_x.transpose()) + 
             grad_y * Eigen::SparseMatrix<double>(grad_y.transpose()) + 
             grad_z * Eigen::SparseMatrix<double>(grad_z.transpose());
        qn = -r_x*grad_x - r_y*grad_y - r_z*grad_z;
        qn.coeffRef(numState-Model::numOrder+1,0) = -qVTheta;
        sp::colMajor::setBlock(Qk, Qn, numState*horizon_, numState*horizon_);
        // sp::colMajor::setRows(qk, qn, numState*horizon_);
        // todo: I do not understand why setRows not work here but well in other places...
        sp::colMajor::setBlock(qk,qn,numState*horizon_,0);
    }
    void MpcSolver::calculateConstrains(Eigen::SparseMatrix<double> &stateTmp, int horizon_){
        assert(stateTmp.rows() == numState);
        double theta_ = stateTmp.coeffRef(numState-Model::numOrder,0);
        double theta = std::fmod(theta_+map.thetaMax, map.thetaMax);
        // double theta = theta_>map.thetaMax ? map.thetaMax:theta_;
        int numConstrains = 0;
        Polyhedron chosenPoly;
        if (theta < 1e-6){
            theta = 1e-6;
        }
        Eigen::Vector3d position, tangentLine;
        map.getGlobalCommand(theta, position, tangentLine);

        // find the max Polyhedron intersecting theta face
        double maxArea = 0;
        // cout << "theta: " << stateTmp.coeffRef(7,0) << endl;
        // find tunnel for this horizon: 
        for (int i=0; i < map.corridor.polys.size(); ++i){
            bool inThisPoly = false;
            for (int k=0; k<map.corridor.polys[i].starter.size(); ++k){
                bool in = theta >= map.corridor.polys[i].starter[k] && theta <= map.corridor.polys[i].ender[k];
                inThisPoly = inThisPoly || in ;
            }
            if ( inThisPoly ){
                // std::cout << "polyindex: " << i << std::endl;
                // cout << "pos: " << position.transpose() << endl;
                // cout << "tan: " << tangentLine.transpose() << endl;
                // cout << "i: " << i << endl;
                map.corridor.FindPolygon(position, tangentLine, i);
                if(maxArea < map.corridor.tunnelArea){
                    maxArea = map.corridor.tunnelArea;
                    chosenPoly = map.corridor.tunnel;
                }
                // std::cout << "area: " << maxArea << endl;
            }
        }
        displayPtr->displayOneTunnel(horizon_, theta);
        numConstrains = chosenPoly.rows();

        Cn.resize(numConstrains, numState);
        Cnb.resize(numConstrains, 1);
        for (int i=0; i<numConstrains; ++i){
            for (int j=0; j<Model::numDimention; ++j){
                Cn.coeffRef(i,Model::numOrder*j) = chosenPoly(i,j);
            }
            Cnb.coeffRef(i,0) = chosenPoly(i,3);
        }
        sp::colMajor::addBlock(Ck, Cn);
        sp::colMajor::addRows(Ckb, Cnb);
    }
    int MpcSolver::solveMpcQp(Eigen::SparseMatrix<double> &stateTmp){
        clock_t t1, t2;
        // t1 = clock();
        // t2 = clock();
        // std:: cout << "first time : " << (double)(t2 - t1)/CLOCKS_PER_SEC << endl;
        Eigen::SparseMatrix<double> state = stateTmp;
        // Vector3d posNow(stateTmp.coeffRef(0,0), 
        //              stateTmp.coeffRef(Model::numOrder,0), 
        //              stateTmp.coeffRef(2*Model::numOrder,0));
        // double nearestTheta = map.findNearestTheta(state.coeffRef(3*Model::numOrder,0), posNow);
        // double nearestTheta = map.findNearestTheta(posNow);
        // state.coeffRef(3*Model::numOrder,0) = nearestTheta;
        Eigen::SparseMatrix<double> inputPredict1toN = inputPredict.block(0,1,numInput,horizon-1);
        sp::colMajor::setCols(inputPredict, inputPredict1toN, 1);
        int solveStatus = 1;

        Ck.resize(0, 0);
        Ckb.resize(0,0);
        
        // t1 = clock();
        displayPtr->displayDrone(state);
        displayPtr->displayTheta(state);
        double theta = 0;
        // displayPtr->clearTunnels();
        for (int horizonI=0; horizonI<horizon; ++horizonI){
            if(initStatus){
                theta = model.ts*horizon;
            }
            else{
                theta = state.coeffRef(numState-Model::numOrder, 0);
            }
            calculateCost(theta, horizonI);
            if (corridorConstrains){
                calculateConstrains(state, horizonI);
            }
            state = model.Ad * state + model.Bd * inputPredict.col(horizonI);
        }
        // t2 = clock();
        // std:: cout << "calculate cost : " << (double)(t2 - t1)/CLOCKS_PER_SEC << endl;
        // t1 = clock();
        Q = BBT * Qk * BB + In;
        c = BBT * Qk*AA*stateTmp + BBT * qk;
        // t2 = clock();
        // std:: cout << "boundary : " << (double)(t2 - t1)/CLOCKS_PER_SEC << endl;
        // state lower and upper constrains : 
        // t1 = clock();

        C = BB;
        
        // ! set terminal velocity constrain
        Vector3d pos ,vel;
        map.getGlobalCommand(theta, pos ,vel);
        xu.coeffRef(numState*(horizon-1)+1, 0) =  1*fabs(vel(0));
        xu.coeffRef(numState*(horizon-1)+4, 0) =  1*fabs(vel(1));
        xu.coeffRef(numState*(horizon-1)+7, 0) =  1*fabs(vel(2));
        xl.coeffRef(numState*(horizon-1)+1, 0) = -1*fabs(vel(0));
        xl.coeffRef(numState*(horizon-1)+4, 0) = -1*fabs(vel(1));
        xl.coeffRef(numState*(horizon-1)+7, 0) = -1*fabs(vel(2));
        // xu.coeffRef(numState*(horizon-1)+10, 0) = 1.1;
        cupp = xu - SparseMatrix<double>(AA*stateTmp);
        clow = xl - SparseMatrix<double>(AA*stateTmp);
        // C.resize(0,0);
        // t2 = clock();
        // std:: cout << "boundary b : " << (double)(t2 - t1)/CLOCKS_PER_SEC << endl;
        // add real constrains
        // t1 = clock();
        if (Ck.rows() != 0){
            // lower bound should be quite small
            Eigen::SparseMatrix<double> zeroRows(Ck.rows(),1);
            for (int i=0; i<Ck.rows(); ++i){
                zeroRows.coeffRef(i, 0) = -1e10;
            }
            sp::colMajor::addRows(clow, zeroRows);
            zeroRows = - Ck * AA * stateTmp - Ckb;
            sp::colMajor::addRows(cupp, zeroRows);
            SparseMatrix<double> Ck2 = Ck * BB;
            sp::colMajor::addRows(C, Ck2);
            sp::colMajor::addRows(C, Inu);
            sp::colMajor::addRows(cupp, uu);
            sp::colMajor::addRows(clow, ul);
        }
        // t2 = clock();
        // std:: cout << "constrains : " << (double)(t2 - t1)/CLOCKS_PER_SEC << endl;
        // printMatrices();
        clock_t t_osqp_start = clock();
        osqpInterface.updateMatrices(Q, c, A, b, C, clow, cupp, ul, uu);
        osqpInterface.solveQP();
        solveStatus = osqpInterface.solveStatus();
        // if (solveStatus == OSQP_PRIMAL_INFEASIBLE){
        //     osqpInterface.updateMatrices(Q, c, A, b, C, clow, cupp, ul, uu, 0);
        //     osqpInterface.solveQP();
        //     solveStatus = osqpInterface.solveStatus();
        // }
        // cout << "solveStatus: " << solveStatus << endl;
        auto solPtr = osqpInterface.solPtr();
        clock_t t_osqp_over = clock();
        double time = (double)(t_osqp_over - t_osqp_start) / CLOCKS_PER_SEC;
        // cout << "osqp time: " << time << ", ";
        // printMatrices();
        if (solveStatus == OSQP_SOLVED){
            // cout << "Solved! " << endl;
            state = stateTmp;
            for (int i=0; i<horizon; ++i){
                for (int j=0; j<numInput; ++j){
                    inputPredict.coeffRef(j,i) = solPtr->x[i*numInput+j];
                }
                state = model.Ad * state + model.Bd * inputPredict.col(i);
                sp::colMajor::setCols(statePredict, state, i);
            }
        }
        else {
            // cout << "failed: " << solveStatus << endl;
            // printMatrices();

            sp::colMajor::setCols(inputPredict, inputPredict1toN, 0);
            SparseMatrix<double> statePredict1toN = statePredict.block(0,1,numState,horizon-1);
            sp::colMajor::setCols(statePredict, statePredict1toN, 0);
        }
        
        if(initStatus){
            initStatus = false;
        }
        displayPtr->displayPredict(statePredict);
        if(solveStatus == OSQP_SOLVED) 
            return 0; // 0 for solved
        else 
            return 1;
    };
    void MpcSolver::printMatrices(){
        cout << "Q:\n" << Q << endl;
        cout << "c:\n" << c << endl;
        cout << "A:\n" << A << endl;
        cout << "b:\n" << b << endl;
        cout << "C:\n" << C << endl;
        cout << "clow:\n" << clow << endl;
        cout << "cupp:\n" << cupp << endl;
        cout << "ul:\n" << ul << endl;
        cout << "uu:\n" << uu << endl;
    }
} // namespace ft
