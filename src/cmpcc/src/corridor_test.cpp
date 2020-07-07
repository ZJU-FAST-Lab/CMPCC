#include <corridor.h>
#include <map.h>
#include <Eigen/Core>
#include <iostream>

int main(void){
    using namespace ft;
    using namespace std;
    Map map;
    for (int i=0; i<map.corridor.polys.size(); ++i){
        std::cout << i << ": ";
        for (int j=0; j<map.corridor.polys[i].starter.size(); ++j){
            std::cout << map.corridor.polys[i].starter[j] << ",";
            std::cout << map.corridor.polys[i].ender[j];
            std::cout << "|";
        }
        std::cout << std::endl;
    }
    
    // double theta = 2;
    // Eigen::MatrixXd thetaPoint = map.trajPoint(theta);
    // std::cout << map.corridor.Cn[1]*thetaPoint-map.corridor.dn[1] << std::endl;
    // cout << "thetaMax: " << map.thetaMax << endl;
    return 0;
}
