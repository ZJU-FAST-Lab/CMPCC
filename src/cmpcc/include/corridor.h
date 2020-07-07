#ifndef PROJECT_CORRIDOR_H
#define PROJECT_CORRIDOR_H
#include <Eigen/Core>
#include <yaml-cpp/yaml.h>

namespace ft{
    struct Poly{
        std::vector<Eigen::Vector3d> points;
        std::vector<Eigen::Vector3d> normals;
        std::vector<double> starter, ender;
    };

    struct PolyLine{
        Eigen::Matrix<double, 1, 6> line;
        std::vector<Eigen::Vector3d> points;  // intersecting point
        std::vector<int> intersecting_line_id;  // 
    };

    typedef Eigen::MatrixX4d Polyhedron;

    class Corridor{
    public:
        std::vector<Poly> polys;
        std::vector<Eigen::MatrixXd> Cn;
        std::vector<Eigen::MatrixXd> dn;
        std::vector<Eigen::MatrixXd> dn_neg;

        // tmp variables:
        Polyhedron tunnel;
        double tunnelArea;

        Corridor(/* args */);

        void FindPolygon(const Eigen::Vector3d position, const Eigen::Vector3d tangent_line, const int corridor_id);
    };

}//namespace ft
#endif
