#include <ctime>
#include <iostream>
#include <Eigen/Geometry>
#include <ros/package.h>
#include "corridor.h"

using namespace std;

namespace ft{

    Corridor::Corridor(){
        std::string path = ros::package::getPath("cmpcc")+"/config/corridor.yaml";
        YAML::Node polyData = YAML::LoadFile(path);
        Eigen::Vector3d point = Eigen::MatrixXd::Zero(3,1);
        Eigen::Vector3d normal = Eigen::MatrixXd::Zero(3,1);
        int numPolys = polyData["polyhedrons"].size();
        int numPoints;
        double x,y,z;
        Eigen::MatrixXd C,d;
        for(int i=0; i < numPolys; ++i){
            numPoints = polyData["polyhedrons"][i]["points"].size();
            Poly poly;
            C = Eigen::MatrixXd::Zero(numPoints,3);
            d = Eigen::MatrixXd::Zero(numPoints,1);
            for(int j=0; j < numPoints; ++j){
                x = polyData["polyhedrons"][i]["points"][j]["x"].as<double>();
                y = polyData["polyhedrons"][i]["points"][j]["y"].as<double>();
                z = polyData["polyhedrons"][i]["points"][j]["z"].as<double>();
                point << x,y,z;
                x = polyData["polyhedrons"][i]["normals"][j]["x"].as<double>();
                y = polyData["polyhedrons"][i]["normals"][j]["y"].as<double>();
                z = polyData["polyhedrons"][i]["normals"][j]["z"].as<double>();
                normal << x,y,z;
                // point = point + 10 * normal.normalized();
                poly.points.push_back(point);
                poly.normals.push_back(normal);
                C.block(j,0,1,3) = normal.transpose();
                d(j) = normal.transpose() * point;
            }
            polys.push_back(poly);
            Cn.push_back(C);
            dn.push_back(d);
            dn_neg.push_back(-d);
        }
    }

    void Corridor::FindPolygon(const Eigen::Vector3d position, const Eigen::Vector3d tangent_line, const int corridor_id)
    {
        clock_t t1, t2;
        t1 = clock();

        assert((Cn[corridor_id] * position - dn[corridor_id]).maxCoeff() < 0);
        for(int i=0; i<polys.size(); ++i){
            assert(polys[i].starter.size() > 0);
        }

        /*  Note
        The index system of the code is extremely complex, because there are too many elements and relationships.
        @param corridor_id              The corridor consists of polyhedrons in a string, this index will find a specific one.
        @param intersecting_line_id     Index of a line during intersecting points calculation.
        @param final_line_id_table      The set of lines(std::vector<int>) that form the finnal polygon. 
        @param forward_point_id         Index of a point on a specific line.
        @param candidate_next_line_id_table       The set of lines(std::vector<int>) that may be part of the polygon and will get further checks. 
        @param current_line_id          Index of a line.
        @param current_forward_point_id Index of a point on a specific line.
        @param current_backward_point_id    Index of a point on a specific line.
        */

        /* Formulation of the line parameterized by t
            {x=x0+m*t
            {y=y0+n*t
            {z=z0+p*t 
            line := {x0, y0, z0, m, n, p}*/  
        PolyLine poly_line;
        std::vector<PolyLine> poly_lines;
        //"ref_plane := A*x+B*y+C*z+D=0" is the plane that is perpendicular to the given line at the given position
        Eigen::Matrix<double,1,3> ref_plane_ABC(tangent_line(0), tangent_line(1), tangent_line(2));
        double ref_plane_D = - position.transpose() * tangent_line;

        // step 1: find all the intersecting lines.
        for ( int id = 0; id < Cn[corridor_id].rows(); id++ )
        {
            Eigen::Vector3d vec_a = ref_plane_ABC;
            Eigen::Vector3d vec_b = Cn[corridor_id].row(id);

            if ( (vec_a.normalized() - vec_b.normalized()).norm() < 1e-5 || (vec_a.normalized() - vec_b.normalized()).norm() > 2-1e-5) // parallel, skip
                continue;

            Eigen::Vector3d vec_line = vec_a.cross( vec_b );  // vec_a and vec_b mast be Eigen::Vector3d
            vec_line.normalize();
            Eigen::Vector3d p_on_ref = - vec_a * (ref_plane_D / (vec_a.norm() * vec_a.norm() ));
            Eigen::Vector3d vec_on_ref = vec_line.cross(vec_a);
            double t = - ( vec_b(0)*p_on_ref(0) + vec_b(1)*p_on_ref(1) + vec_b(2)*p_on_ref(2) + dn_neg[corridor_id](id) ) 
                       / (vec_b(0)*vec_on_ref(0) + vec_b(1)*vec_on_ref(1) + vec_b(2)*vec_on_ref(2));
            Eigen::Vector3d point_line( p_on_ref(0) + vec_on_ref(0) * t, p_on_ref(1) + vec_on_ref(1) * t, p_on_ref(2) + vec_on_ref(2) * t);
            poly_line.line.block(0,0,1,3) = point_line.transpose();
            poly_line.line.block(0,3,1,3) = vec_line.transpose();
            poly_lines.push_back(poly_line);
        }

        // step 2: find all the intersecting points of the lines.
        for ( int i = 0; i < poly_lines.size(); i++ )
        {
            for ( int j = i+1; j < poly_lines.size(); j++ )
            {
                // A*t = B ==> t = ?
                Eigen::Matrix<double,2,1> t;
                Eigen::Matrix<double,2,2> A, A_;
                Eigen::Matrix<double,2,1> B;
                int flag = 0;

                for ( int k = 0; k < 3; k++ )
                {
                    const int idx_ord[] = {0,1,0,1,2,2};
                    A << poly_lines[i].line(idx_ord[k]+3), - poly_lines[j].line(idx_ord[k]+3), poly_lines[i].line(idx_ord[k+3]+3), - poly_lines[j].line(idx_ord[k+3]+3);
                    if( fabs(A.determinant()) > 1e-5 )  // numerical stability
                    {
                        // calculate t
                        B << (poly_lines[j].line(idx_ord[k]) - poly_lines[i].line(idx_ord[k])), (poly_lines[j].line(idx_ord[k+3]) - poly_lines[i].line(idx_ord[k+3]));
                        A_ << A(1,1), -A(0,1), -A(1,0), A(0,0);
                        t = 1 / ( A(0,0)*A(1,1) - A(0,1)*A(1,0) ) * A_ * B;

                        // get point
                        Eigen::Vector3d point;
                        point << poly_lines[i].line(0) + poly_lines[i].line(3) * t(0,0),
                                 poly_lines[i].line(1) + poly_lines[i].line(4) * t(0,0),
                                 poly_lines[i].line(2) + poly_lines[i].line(5) * t(0,0);
                        poly_lines[i].points.push_back(point);
                        poly_lines[i].intersecting_line_id.push_back(j);

                        point << poly_lines[j].line(0) + poly_lines[j].line(3) * t(1,0),
                                 poly_lines[j].line(1) + poly_lines[j].line(4) * t(1,0),
                                 poly_lines[j].line(2) + poly_lines[j].line(5) * t(1,0);
                        poly_lines[j].points.push_back(point);
                        poly_lines[j].intersecting_line_id.push_back(i);
                        
                        break;
                    }
                }
            } 
        }

        // step 3: find the polygon
            //step 3.1: find the closest line to the point 
        std::vector<int> final_line_id_table = {-1};  // IDs of the lines that form the final polygen
        double min_dist = abs(INFINITY);
        Eigen::Vector3d pedal(INFINITY,INFINITY,INFINITY);
        for ( int i = 0; i < poly_lines.size(); i++ )
        {
            // calculate distace from a point to a line
            double x0,y0,z0, x1,y1,z1, xc,yc,zc, m,n,p, t, d;
            x0 = position(0);
            y0 = position(1);
            z0 = position(2);
            x1 = poly_lines[i].line(0,0);
            y1 = poly_lines[i].line(0,1);
            z1 = poly_lines[i].line(0,2);
            m = poly_lines[i].line(0,3);
            n = poly_lines[i].line(0,4);
            p = poly_lines[i].line(0,5);
            t = ( m*(x0-x1) + n*(y0-y1) + p*(z0-z1)) / (m*m + n*n + p*p);
            xc = m*t + x1;
            yc = n*t + y1;
            zc = p*t + z1;
            d = sqrt( (x0-xc)*(x0-xc) + (y0-yc)*(y0-yc) + (z0-zc)*(z0-zc) );
            if ( d < min_dist )
            {
                min_dist = d;
                final_line_id_table[0] = i;
                pedal << xc, yc, zc;
            }
        }
        if( isinf(min_dist) || final_line_id_table[0] == -1 )
        {
            cout << "\033[31m[corridor.cpp]ERROR CODE: 0x01\033[0m" << endl;
            assert(false);
        }
        
        /*  step 3.2: Search line by line
                step 3.2.1: use cross product to distinguish if a point is on the clockwise side.
                step 3.2.2: find the closest point on the right side.
                step 3.2.3: segment the linked line on the clockwise side.
                step 3.2.4: compare and find the line segment with smallest angle.
                step 3.2.5: continue this loop until meeting the original line. */
        Eigen::Vector3d vec_ret, vec_l, vec_r; // vec_ret = vec_l x vec_r.
        double dist_min = abs(INFINITY);
        int forward_point_id = -1;
        std::vector<int> candidate_next_line_id_table;
        vector<Eigen::Vector3d> vertexs;      // record vertexs for calculating area
        // find the first intersection point.
        for ( int i = 0; i < poly_lines[final_line_id_table[0]].points.size(); i++ )
        {
            vec_l = poly_lines[final_line_id_table[0]].points[i] - pedal;
            vec_r = position - pedal;
            vec_ret = vec_l.cross(vec_r);
            if( vec_ret.dot(tangent_line) > 0) // same direction
            {
                double dist = ( poly_lines[final_line_id_table[0]].points[i] - pedal ).norm();
                // sometimes more than two lines intersect at one same point.
                if( dist < dist_min )
                {
                    dist_min = dist;
                    candidate_next_line_id_table.clear();
                    candidate_next_line_id_table.push_back(poly_lines[final_line_id_table[0]].intersecting_line_id[i]);
                    forward_point_id = i;
                }
                else if( dist - dist_min < 1e-100 ) // Think of them as equals
                {
                    candidate_next_line_id_table.push_back(poly_lines[final_line_id_table[0]].intersecting_line_id[i]);
                }
                //cout << "dist = " << dist << " dist_min = " << dist_min << " candidate_next_line_id_table = " << candidate_next_line_id_table[0] << endl;
            }
        }
        if( isinf(dist_min) || candidate_next_line_id_table.empty() )
        {
            cout << "\033[31m[corridor.cpp]ERROR CODE: 0x02\033[0m" << endl;
            assert(false);
        }
        vertexs.push_back( poly_lines[final_line_id_table[0]].points[forward_point_id] );


        // start loop
        Eigen::Vector3d last_line_backward_point = pedal;
        Eigen::Vector3d last_line_forward_point = poly_lines[final_line_id_table[0]].points[forward_point_id];
        bool flag_find_a_polygon = false;
        bool flag_dead_loop = false;
        int iter_count = 0;
        int current_line_id = -1;
        int current_forward_point_id = -1;
        int current_backward_point_id = -1;
        while( ! flag_find_a_polygon && ! flag_dead_loop )
        {
            // find the line segment with smallest angle.
            double max_cos = -1.0;
            for ( int i = 0; i < candidate_next_line_id_table.size(); i++ )
            {
                vec_l = poly_lines[candidate_next_line_id_table[i]].line.block(0,3,1,3).transpose();
                vec_r = last_line_backward_point - last_line_forward_point;
                vec_ret = vec_l.cross(vec_r);
                if (vec_ret.dot(tangent_line) < 0 ) // inverse direction
                {
                    vec_l = - vec_l;
                }
                double cos_l_r = vec_l.dot(vec_r) / ( vec_l.norm() * vec_r.norm() ); // cos_l_r from +1 -> -1
                if( cos_l_r > max_cos ) // The bigger cos, the smaller angle
                {
                    max_cos = cos_l_r;
                    current_line_id = candidate_next_line_id_table[i];
                }
            }
            if( max_cos == -1.0 )
            {
                cout << "\033[31m[corridor.cpp]ERROR CODE: 0x03\033[0m" << endl;
                assert(false);
            }

            // if ( candidate_next_line_id_table.size() >= 2 )
            // {
            //     cout << "current_line_id=" << current_line_id << endl;
            //     for ( int i=0; i<candidate_next_line_id_table.size(); i++ )
            //     {
            //         cout << candidate_next_line_id_table[i] << endl;
            //     }
            // }

            // delete the intersection information of all the redundant lines permanently
            for ( int i = 0; i < candidate_next_line_id_table.size(); i++ )
            {
                if ( candidate_next_line_id_table[i] != current_line_id )
                {
                    // printf("candidate_next_line_id_table[i]=%d\n",candidate_next_line_id_table[i]);
                    int redundant_line_id = candidate_next_line_id_table[i];

                    poly_lines[redundant_line_id].intersecting_line_id.clear();
                    poly_lines[redundant_line_id].points.clear();
                    
                    for ( int j=0; j<poly_lines.size(); j++ )
                    {
                        if ( j == redundant_line_id ) 
                            continue;

                        for ( int k=0; k<poly_lines[j].intersecting_line_id.size(); k++ )
                        {
                            if ( poly_lines[j].intersecting_line_id[k] == redundant_line_id )
                            {
                                poly_lines[j].intersecting_line_id.erase( poly_lines[j].intersecting_line_id.begin() + k );
                                poly_lines[j].points.erase( poly_lines[j].points.begin() + k );
                                break;
                            }
                        }
                    }
                }
            }
            // cout << endl;

            // successfully finish detect
            if( current_line_id == final_line_id_table[0] )
            {
                flag_find_a_polygon = true;  // just a symbol for better reading, but not take any effect
                break;
            }

            // dead loop detect
            if( iter_count > poly_lines.size() )
            {
                flag_dead_loop = true;  // just a symbol for better reading, but not take any effect
                cout << "\033[33m[corridor.cpp]ERROR: dead loop!\033[0m" << endl;

                cout << "position=" << position.transpose() << endl << " tangent_line=" << tangent_line.transpose() << endl;

                for ( int i = 0; i < poly_lines.size(); i++ )
                {
                    // cout << "i = " << i << " ## " << poly_lines[i].line << endl;
                    cout.precision(15);
                    cout << i << " " << poly_lines[i].line << endl;
                }
                cout << endl;

                for ( int i=0; i<final_line_id_table.size(); i++ )
                {
                    cout << final_line_id_table[i] << " ";
                }
                cout << endl;
                cout << endl;

                // for ( int i = 0; i < poly_lines.size(); i++ )
                // {
                //     for ( int j = 0; j < poly_lines[i].points.size(); j++)
                //     {
                //         cout << poly_lines[i].intersecting_line_id[j] << " " << poly_lines[i].points[j].transpose() << endl;
                //     }
                //     cout << endl;
                // }
                // cout << endl;

                assert(false);
            }

            // find the intersection id of current line to the last line
            for ( int i = 0; i < poly_lines[current_line_id].intersecting_line_id.size(); i++ )
            {
                if ( poly_lines[current_line_id].intersecting_line_id[i] == final_line_id_table.back() )
                {
                    current_backward_point_id = i;
                }
            }

            // cout << "cur_id=" << current_line_id << endl;

            // find next lines
            dist_min = abs(INFINITY);
            candidate_next_line_id_table.clear();
            for ( int i = 0; i < poly_lines[current_line_id].points.size(); i++ )
            {
                if ( i == current_backward_point_id ) 
                    continue;

                vec_l = poly_lines[current_line_id].points[i] - poly_lines[current_line_id].points[current_backward_point_id];
                vec_r = last_line_backward_point - last_line_forward_point;
                vec_ret = vec_l.cross(vec_r);
                if( vec_ret.dot(tangent_line) > 0) // same direction
                {
                    double dist = ( poly_lines[current_line_id].points[i] - poly_lines[current_line_id].points[current_backward_point_id] ).norm();
                    // cout << current_line_id << " " << poly_lines[current_line_id].intersecting_line_id[i] << " " << dist << " " << dist_min << endl;
                    // sometimes more than two lines intersect at one same point.
                    // cout << "first_id=" << final_line_id_table[0] << " cur_id=" << current_line_id << " id=" << poly_lines[current_line_id].intersecting_line_id[i] << " dist=" << dist << " dist_min=" << dist_min << endl;
                    constexpr double CLOSE_MARGIN = 1e-3; // 1mm
                    if( dist <= dist_min - CLOSE_MARGIN ) // small enough
                    {
                        dist_min = dist;
                        current_forward_point_id = i;
                        if ( candidate_next_line_id_table.size() <= 1 )
                        {
                            candidate_next_line_id_table.clear();
                        }
                        else
                        {
                            std::vector<int> candidate_next_line_id_table_temp;
                            for( int j = 0; j < candidate_next_line_id_table.size(); j++ )
                            {
                                for ( int k = 0; k < poly_lines[current_line_id].intersecting_line_id.size(); k++ )
                                {
                                    if ( poly_lines[current_line_id].intersecting_line_id[k] == candidate_next_line_id_table[j] )
                                    {
                                        double dist2 = (poly_lines[current_line_id].points[k] - poly_lines[current_line_id].points[current_backward_point_id]).norm();
                                        if ( abs(dist2 - dist_min) < CLOSE_MARGIN )
                                        {
                                            candidate_next_line_id_table_temp.push_back(candidate_next_line_id_table[j]);
                                        }
                                    }
                                }
                            }
                            candidate_next_line_id_table = candidate_next_line_id_table_temp;
                        }
                        candidate_next_line_id_table.push_back(poly_lines[current_line_id].intersecting_line_id[i]);
                        
                    }
                    else if( dist - dist_min < CLOSE_MARGIN ) // Some points closed to each other will be taken as candidates and the one of smallest angle will be select at the start of next iteration.
                    {
                        // cout << "AAAAAAAAAAAAAAAAAAAAAAAAAAAA" << endl;
                        // cout << "AAAAAAAAAAAAAAAAAAAAAAAAAAAA" << endl;
                        // cout << "AAAAAAAAAAAAAAAAAAAAAAAAAAAA" << endl;
                        // cout << "AAAAAAAAAAAAAAAAAAAAAAAAAAAA" << endl;
                        // cout << "AAAAAAAAAAAAAAAAAAAAAAAAAAAA" << endl;
                        candidate_next_line_id_table.push_back(poly_lines[current_line_id].intersecting_line_id[i]);
                    }
                }
            }
            if( isinf(dist_min) || candidate_next_line_id_table.empty() )
            {
                cout << "isinf(dist_min)=" << isinf(dist_min) << " candidate_next_line_id_table.empty()=" << candidate_next_line_id_table.empty() << endl;
                cout << "\033[31m[corridor.cpp]ERROR CODE: 0x05\033[0m" << endl;
                assert(false);
            }
            vertexs.push_back( poly_lines[current_line_id].points[current_forward_point_id] );

            last_line_backward_point = poly_lines[ current_line_id ].points[ current_backward_point_id ];
            last_line_forward_point = poly_lines[ current_line_id ].points[ current_forward_point_id ];
            final_line_id_table.push_back(current_line_id);
            iter_count++;

            // cout << "current_line_id=" << current_line_id << " iter_count=" << iter_count << endl;
            // for ( int i=0; i<candidate_next_line_id_table.size(); i++ )
            // {
            //     cout << "i=" << i << " candidate_id[i]=" << candidate_next_line_id_table[i] << endl;
            // }
            // cout << "--------------" << endl;
        }
        // cout << endl << endl;

        // calculate the area
        double area = 0;
        for ( int i = 0; i < vertexs.size(); i++)
        {
            vec_l = vertexs[i] - position;
            if ( i == vertexs.size() - 1 )
            {
                vec_r = vertexs[0] - position;
            }
            else
            {
                vec_r = vertexs[i+1] - position;
            }
            vec_ret = vec_l.cross(vec_r);
            area += vec_ret.norm() / 2.0;
        }

        // generate a polyhedron and return
        ft::Polyhedron polyhedron(final_line_id_table.size(),4);
        Eigen::Vector3d plane_normal, point_on_plane;
        double D;
        for ( int i = 0; i < final_line_id_table.size(); i++)
        {
            vec_l = tangent_line;
            vec_r = poly_lines[final_line_id_table[i]].line.block(0,3,1,3).transpose();
            plane_normal = vec_l.cross(vec_r);
            point_on_plane = poly_lines[final_line_id_table[i]].line.block(0,0,1,3).transpose();
            D = - point_on_plane.transpose() * plane_normal;
            if( position.transpose() * plane_normal + D > 0 )
            {
                plane_normal = - plane_normal;
                D = -D;
            }
            polyhedron.row(i) << plane_normal.transpose(), D;
            polyhedron.row(i) = polyhedron.row(i).normalized();
        }

        if ( ! flag_find_a_polygon )
        {
            cout << "\033[31m[corridor.cpp]ERROR CODE: 0x06\033[0m" << endl;
            assert(false);
        }

        t2 = clock();
        double totaltime=(double)(t2-t1)/CLOCKS_PER_SEC;
	    // cout<<"Total time: "<<totaltime*1000<<endl;		//ms

        tunnel = polyhedron;
        // cout << "tunnel"<< corridor_id << ": " << endl << tunnel << endl;
        tunnelArea = area;

        // printf("AAAAAAAAAAAAAA\n");
        // printf("AAAAAAAAAAAAAA\n");
        // printf("AAAAAAAAAAAAAA\n");
    }
}//namespace ft
