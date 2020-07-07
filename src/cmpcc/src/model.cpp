#include "model.h"
#include <iostream>

using namespace Eigen;
using namespace std;
namespace ft{
    Model::Model (double T){
        Ad.resize(numState,numState);
        Bd.resize(numState,numInput);
        std::vector<Eigen::Triplet<double>> triplets;
        for (int i=0; i<numState; ++i){
            triplets.push_back(Triplet<double>(i,i,1));
            if ((i+1) % numOrder){
                triplets.push_back(Triplet<double>(i,i+1,T));
            }
        }
        Ad.setFromTriplets(triplets.begin(), triplets.end());
        triplets.clear();
        for (int i=0; i<numInput; ++i){
            triplets.push_back(Triplet<double>((i+1)*numOrder-1,i,T));
        }
        Bd.setFromTriplets(triplets.begin(), triplets.end());
    }

}// namespace ft
