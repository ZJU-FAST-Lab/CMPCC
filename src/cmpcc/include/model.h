//
// Created by soviet on 19-11-17.
//

#ifndef PROJECT_MODEL_H
#define PROJECT_MODEL_H

#include <Eigen/Sparse>

namespace ft{

// class Model
    class Model{
        public:

        static constexpr unsigned long freq = 20; // frequency of discretization
        static constexpr double ts = 1.0/freq;
        static constexpr int numOrder = 3;        // number of orders
        static constexpr int numDimention = 3;    // number of dimentions
        static constexpr int numState = numOrder*(numDimention+1); // number of states
        static constexpr int numInput = numDimention+1; // number of inputs

        Eigen::SparseMatrix<double> Ad;
        Eigen::SparseMatrix<double> Bd;

        Model (double T = ts);
    };

}//namespace ft -> flight_tunnel



#endif //PROJECT_MODEL_H
