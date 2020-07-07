#include "map.h"
#include <ros/package.h>
#include <ros/ros.h>

using namespace std;
using namespace Eigen;
namespace ft{
    Map::Map(){
        std::string path = ros::package::getPath("cmpcc")+"/config/traj.yaml";
        YAML::Node node = YAML::LoadFile(path);
        num_segment  = node["num_segment" ].as<int> ();
        traj_order = 5;
        K_data = node["K"].as <std::vector<double>>();
        K_max = node["K_max"].as <int>();
        range  = node["range" ].as < std::vector < double >> ();
        long secs = node["final_time"]["secs"].as<long>() - node["start_time"]["secs"].as<long>();
        long nsecs = node["final_time"]["nsecs"].as<long>() - node["start_time"]["nsecs"].as<long>();
        global_traj_time = secs + nsecs*1e-9;
        thetaMax = global_traj_time;
        vector<double> coef_x = node["coef_x"].  as < vector < double >> ();
        vector<double> coef_y = node["coef_y"].  as < vector < double >> ();
        vector<double> coef_z = node["coef_z"].  as < vector < double >> ();
        vector<double> tt     = node["time"].    as < vector < double >> ();
        vector<double> ta     = node["time_acc"].as < vector < double >> ();
        vector<double> a      = node["a"].       as < vector < double >> ();
        vector<double> b      = node["b"].       as < vector < double >> ();
        vector<double> s      = node["s"].       as < vector < double >> ();
        int seg_num = K_data.size();
        a_data   = MatrixXd::Zero(seg_num, K_max);
        b_data   = MatrixXd::Zero(seg_num, K_max + 1);
        s_data   = MatrixXd::Zero(seg_num, K_max + 1);
        time     = MatrixXd::Zero(seg_num, K_max);
        time_acc = MatrixXd::Zero(seg_num, K_max);
        coef = Eigen::MatrixXd::Zero(num_segment, 3*(traj_order + 1) );
        bezier_basis.setFixedOrder(traj_order);
        int poly_num = traj_order + 1;
        for (int idx = 0; idx < num_segment; ++idx){   
            for (int j = 0; j < poly_num; ++j)
            {
                coef(idx, 0 * poly_num + j) = coef_x[idx * poly_num + j];
                coef(idx, 1 * poly_num + j) = coef_y[idx * poly_num + j];
                coef(idx, 2 * poly_num + j) = coef_z[idx * poly_num + j];
            }
        }
        int K_shift = 0;
        int K_plus_shift = 0;
        for (int i = 0; i < seg_num; i++ )
        {
            for(int j = 0; j < K_data[i] + 1; j++ )
            {   
                if( j < K_data[i] )
                {
                    a_data(i, j) = a[j + K_shift];
                    time    (i, j) = tt[j + K_shift];
                    time_acc(i, j) = ta[j + K_shift];
                }

                b_data(i, j) = b[j + K_plus_shift];
                s_data(i, j) = s[j + K_plus_shift];
            }

            K_shift      += K_data[i];
            K_plus_shift += K_data[i] + 1;
        }
        // corridor
        double theta, maxItem;
        int starter = 0;
        bool inPoly = false;
        Eigen::Vector3d thetaPoint;
        for (int i=0; i<corridor.polys.size(); ++i){
            for (theta=0; theta<thetaMax; theta+=0.001){
                getGlobalCommand(theta, thetaPoint);
                theta_sample.push_back(theta);
                pos_sample.push_back(thetaPoint);
                maxItem = (corridor.Cn[i]*thetaPoint-corridor.dn[i]).maxCoeff();
                if (maxItem < 0 && !inPoly){
                    corridor.polys[i].starter.push_back(theta);
                    inPoly = true;
                }
                if (maxItem > 0 && inPoly){
                    corridor.polys[i].ender.push_back(theta-0.001);
                    inPoly = false;
                }
            }
            if (inPoly){
                corridor.polys[i].ender.push_back(thetaMax);
                inPoly = false;
            }
        }
    }
    double Map::findNearestTheta(double theta, Eigen::Vector3d & position){
        Eigen::Vector3d thetaPoint;
        int index = 0;
        int left = 0;
        int right = theta_sample.size()-1;
        double distance = 0;
        double nearestTheta = 0;
        double distanceMin = 10;
        double error = 10;
        while(fabs(error) > 0.005){
            error = theta_sample[index] - theta;
            if(error > 0){
                right = index;
                index = (index+left)/2;
            }
            else{
                left = index;
                index = (index+right)/2;
            }
        }
        for (int i=((index-1000)>0? index-1000:0); i<(index+1000<theta_sample.size()? index+1000:theta_sample.size()-1); i+=100){
            distance = (position - pos_sample[i]).squaredNorm();
            if(distance < distanceMin){
                distanceMin = distance;
                nearestTheta = theta_sample[i];
                index = i;
            }
        }
        for (int i=((index-100)>0? index-100:0); i<(index+100<theta_sample.size()? index+100:theta_sample.size()-1); i+=10){
            distance = (position - pos_sample[i]).squaredNorm();
            if(distance < distanceMin){
                distanceMin = distance;
                nearestTheta = theta_sample[i];
                index = i;
            }
        }
        for (int i=((index-10)>0? index-10:0); i<(index+10<theta_sample.size()? index+10:theta_sample.size()-1); i+=1){
            distance = (position - pos_sample[i]).squaredNorm();
            if(distance < distanceMin){
                distanceMin = distance;
                nearestTheta = theta_sample[i];
            }
        }
        return nearestTheta;
    };
    double Map::findNearestTheta(Eigen::Vector3d & position){
        Eigen::Vector3d thetaPoint;
        int index;
        double distance = 0;
        double nearestTheta = 0;
        double distanceMin = 10;
        // sampling method for finding the nearest point on the trajectory
        for (int i=0; i<theta_sample.size(); i+=1000){
            distance = (position - pos_sample[i]).squaredNorm();
            if(distance < distanceMin){
                distanceMin = distance;
                nearestTheta = theta_sample[i];
                index = i;
            }
        }
        for (int i=((index-1000)>0? index-1000:0); i<(index+1000<theta_sample.size()? index+1000:theta_sample.size()-1); i+=100){
            distance = (position - pos_sample[i]).squaredNorm();
            if(distance < distanceMin){
                distanceMin = distance;
                nearestTheta = theta_sample[i];
                index = i;
            }
        }
        for (int i=((index-100)>0? index-100:0); i<(index+100<theta_sample.size()? index+100:theta_sample.size()-1); i+=10){
            distance = (position - pos_sample[i]).squaredNorm();
            if(distance < distanceMin){
                distanceMin = distance;
                nearestTheta = theta_sample[i];
                index = i;
            }
        }
        for (int i=((index-10)>0? index-10:0); i<(index+10<theta_sample.size()? index+10:theta_sample.size()-1); i+=1){
            distance = (position - pos_sample[i]).squaredNorm();
            if(distance < distanceMin){
                distanceMin = distance;
                nearestTheta = theta_sample[i];
            }
        }
        return nearestTheta;
    }

    void Map::findRangeForEachPoly(){
        // find range of theta for each poly
        double theta, maxItem;
        int starter = 0;
        bool inPoly = false;
        Eigen::Vector3d thetaPoint;
        for (int i=0; i<corridor.polys.size(); ++i){
            for (theta=0; theta<thetaMax; theta+=0.001){
                getGlobalCommand(theta, thetaPoint);
                maxItem = (corridor.Cn[i]*thetaPoint-corridor.dn[i]).maxCoeff();
                if (maxItem < 0 && !inPoly){
                    corridor.polys[i].starter.push_back(theta);
                    inPoly = true;
                }
                if (maxItem > 0 && inPoly){
                    corridor.polys[i].ender.push_back(theta-0.001);
                    inPoly = false;
                }
            }
            if (inPoly){
                corridor.polys[i].ender.push_back(thetaMax);
                inPoly = false;
            }
        }
    }

    void Map::getGlobalCommand(double t, Vector3d & position){
        t = std::fmod(t, global_traj_time);

        int idx;
        for(idx = 0; idx < num_segment; idx++)
        {   
            int K = K_data[idx];
            if( t  > time(idx, K - 1)){
                t -= time(idx, K - 1);
            }
            else break;
        }

        double t_tmp = t;     
        int grid_num = K_data[idx];

        //ROS_WARN("[Time Optimal Server] publish command, segm index is %d, segm time is %f", idx, t);
        int grid_idx;
        for(grid_idx = 0; grid_idx < K_data[idx]; grid_idx++)
        {
            if (t > time(idx, grid_idx)) continue;
            else
            { 
                if(grid_idx > 0) t -= time(idx, grid_idx - 1);
                else t -= 0.0;

                break;
            }
        }

        //ROS_WARN("[Time Optimal Server] publish command, grid index is %d, grid time is %f", grid_idx, t);
        double delta_t;
        if(grid_idx > 0)
          delta_t = (time(idx, grid_idx) - time(idx, grid_idx - 1));
        else
          delta_t = time(idx, grid_idx) - 0.0;
        
        double delta_s = t * s_step / delta_t;

        double s = s_data(idx, grid_idx) + delta_s;

        Vector3d position_s = bezier_basis.getPos(coef, idx, s/range[idx]) * range[idx]; 
        position   = position_s;
    }

    void Map::getGlobalCommand(double t, Vector3d & position, Vector3d & velocity){
        t = std::fmod(t, global_traj_time);

        int idx;
        for(idx = 0; idx < num_segment; idx++)
        {   
            int K = K_data[idx];
            if( t  > time(idx, K - 1)){
                t -= time(idx, K - 1);
            }
            else break;
        }

        double t_tmp = t;     
        int grid_num = K_data[idx];

        //ROS_WARN("[Time Optimal Server] publish command, segm index is %d, segm time is %f", idx, t);
        int grid_idx;
        for(grid_idx = 0; grid_idx < K_data[idx]; grid_idx++)
        {
            if (t > time(idx, grid_idx)) continue;
            else
            { 
                if(grid_idx > 0) t -= time(idx, grid_idx - 1);
                else t -= 0.0;

                break;
            }
        }

        //ROS_WARN("[Time Optimal Server] publish command, grid index is %d, grid time is %f", grid_idx, t);
        double delta_t;
        if(grid_idx > 0)
          delta_t = (time(idx, grid_idx) - time(idx, grid_idx - 1));
        else
          delta_t = time(idx, grid_idx) - 0.0;
        
        double delta_s = t * s_step / delta_t;

        double s = s_data(idx, grid_idx) + delta_s;

        Vector3d position_s = bezier_basis.getPos(coef, idx, s/range[idx]) * range[idx]; 
        position   = position_s;
        // cout<<"s: "<<s<<", position: "<<position <<", range: "<<range[idx] << ", idx:"<<idx <<endl;

        double s_k   = s_data(idx, grid_idx);
        double s_k_1 = s_data(idx, grid_idx + 1);
        double b_k   = b_data(idx, grid_idx);
        double b_k_1 = b_data(idx, grid_idx + 1);

        Vector3d velocity_s1 = bezier_basis.getVel(coef, idx, s_k   /range[idx]); 
        Vector3d velocity_s2 = bezier_basis.getVel(coef, idx, s_k_1 /range[idx]);

        Vector3d velocity1 = velocity_s1 * sqrt(b_k);
        Vector3d velocity2 = velocity_s2 * sqrt(b_k_1);
        velocity  = velocity1 + (velocity2 - velocity1) * t / delta_t;
    }
    
    void Map::getGlobalCommand(double t, Vector3d & position, Vector3d & velocity, Vector3d & acceleration)
    {   
        t = std::fmod(t, global_traj_time);

        int idx;
        for(idx = 0; idx < num_segment; idx++)
        {   
            int K = K_data[idx];
            if( t  > time(idx, K - 1)){
                t -= time(idx, K - 1);
            }
            else break;
        }

        double t_tmp = t;     
        int grid_num = K_data[idx];

        //ROS_WARN("[Time Optimal Server] publish command, segm index is %d, segm time is %f", idx, t);
        int grid_idx;
        for(grid_idx = 0; grid_idx < K_data[idx]; grid_idx++)
        {
            if (t > time(idx, grid_idx)) continue;
            else
            { 
                if(grid_idx > 0) t -= time(idx, grid_idx - 1);
                else t -= 0.0;

                break;
            }
        }

        //ROS_WARN("[Time Optimal Server] publish command, grid index is %d, grid time is %f", grid_idx, t);
        double delta_t;
        if(grid_idx > 0)
          delta_t = (time(idx, grid_idx) - time(idx, grid_idx - 1));
        else
          delta_t = time(idx, grid_idx) - 0.0;
        
        double delta_s = t * s_step / delta_t;

        double s = s_data(idx, grid_idx) + delta_s;

        Vector3d position_s = bezier_basis.getPos(coef, idx, s/range[idx]) * range[idx]; 
        position   = position_s;
        // cout<<"s: "<<s<<", position: "<<position <<", range: "<<range[idx] << ", idx:"<<idx <<endl;

        double s_k   = s_data(idx, grid_idx);
        double s_k_1 = s_data(idx, grid_idx + 1);
        double b_k   = b_data(idx, grid_idx);
        double b_k_1 = b_data(idx, grid_idx + 1);

        Vector3d velocity_s1 = bezier_basis.getVel(coef, idx, s_k   /range[idx]); 
        Vector3d velocity_s2 = bezier_basis.getVel(coef, idx, s_k_1 /range[idx]);

        Vector3d velocity1 = velocity_s1 * sqrt(b_k);
        Vector3d velocity2 = velocity_s2 * sqrt(b_k_1);
        velocity  = velocity1 + (velocity2 - velocity1) * t / delta_t;

        t = t_tmp;
        for(grid_idx = 0; grid_idx < K_data[idx]; grid_idx++)
        {
            if (t > time_acc(idx, grid_idx)) continue;
            else
            { 
                if(grid_idx > 0) t -= time_acc(idx, grid_idx - 1);
                else t -= 0.0;

                break;
            }
        }
        
        if(grid_idx == grid_num) t -= time_acc(idx, grid_num - 1);

        //ROS_WARN("[Time Optimal Server] publish command, grid index is %d, grid time is %f", grid_idx, t);
        Vector3d velocity_s, acceleration_s, acceleration1, acceleration2;

        double a_k, s_1;
        if( grid_idx == 0 && idx == 0 )
        {   
            s_k   = s_data(idx, 0);
            s_k_1 = s_data(idx, 0 + 1);
            
            a_k   = a_data(idx, 0);
            b_k   = b_data(idx, 0);
            b_k_1 = b_data(idx, 0 + 1);

            s_1 = (s_k + s_k_1 ) / 2.0 / range[idx];
            velocity_s     = bezier_basis.getVel(coef, idx, s_1);
            acceleration_s = bezier_basis.getAcc(coef, idx, s_1) / range[idx];
            acceleration2 = velocity_s * a_k + acceleration_s * (b_k + b_k_1) / 2.0;
            acceleration1 << 0.0, 0.0, 0.0;
            
            acceleration   = acceleration1 + (acceleration2 - acceleration1) * t / time_acc(0, 0); 
        }
        else if( grid_idx == grid_num && idx == (num_segment - 1) )
        {   
            s_k   = s_data(idx, grid_num - 1);
            s_k_1 = s_data(idx, grid_num);
            
            a_k   = a_data(idx, grid_num - 1);
            b_k   = b_data(idx, grid_num - 1);
            b_k_1 = b_data(idx, grid_num    );

            s_1 = (s_k + s_k_1 ) / 2.0 /range[idx];
            velocity_s     = bezier_basis.getVel(coef, idx, s_1);
            acceleration_s = bezier_basis.getAcc(coef, idx, s_1) / range[idx];
            acceleration = velocity_s * a_k + acceleration_s * (b_k + b_k_1) / 2.0;
        }
        else
        {   
            if(grid_idx < grid_num && grid_idx > 0) // take average accleration in a same segment
            {   
                delta_t = (time_acc(idx, grid_idx) - time_acc(idx, grid_idx - 1));
                
                s_k   = s_data(idx, grid_idx - 1);
                s_k_1 = s_data(idx, grid_idx + 0);
                
                a_k   = a_data(idx, grid_idx - 1);
                b_k   = b_data(idx, grid_idx - 1);
                b_k_1 = b_data(idx, grid_idx + 0);

                s_1 = (s_k + s_k_1 ) / 2.0 /range[idx];
                velocity_s     = bezier_basis.getVel(coef, idx, s_1);
                acceleration_s = bezier_basis.getAcc(coef, idx, s_1) / range[idx];
                acceleration1 = velocity_s * a_k + acceleration_s * (b_k + b_k_1) / 2.0;

                s_k   = s_data(idx, grid_idx + 0);
                s_k_1 = s_data(idx, grid_idx + 1);

                a_k   = a_data(idx, grid_idx + 0);
                b_k   = b_data(idx, grid_idx + 0);
                b_k_1 = b_data(idx, grid_idx + 1);              

                s_1 = (s_k + s_k_1 ) / 2.0 /range[idx];
                velocity_s     = bezier_basis.getVel(coef, idx, s_1);
                acceleration_s = bezier_basis.getAcc(coef, idx, s_1) / range[idx];
                acceleration2 = velocity_s * a_k + acceleration_s * (b_k + b_k_1) / 2.0;
                acceleration   = acceleration1 + (acceleration2 - acceleration1) * t / delta_t;   
            }
            else if(grid_idx == grid_num)// take average accleration between two segments
            {   
                delta_t = (time(idx, grid_num - 1) - time_acc(idx, grid_num - 1) + time_acc(idx + 1, 0) );
                
                s_k   = s_data(idx, grid_idx - 1);
                s_k_1 = s_data(idx, grid_idx);
                
                a_k   = a_data(idx, grid_idx - 1);
                b_k   = b_data(idx, grid_idx - 1);
                b_k_1 = b_data(idx, grid_idx);

                s_1 = (s_k + s_k_1 ) / 2.0 /range[idx];
                velocity_s     = bezier_basis.getVel(coef, idx, s_1);
                acceleration_s = bezier_basis.getAcc(coef, idx, s_1) / range[idx];
                acceleration1 = velocity_s * a_k + acceleration_s * (b_k + b_k_1) / 2.0;
                s_k   = s_data(idx + 1, 0);
                s_k_1 = s_data(idx + 1, 1);

                a_k   = a_data(idx + 1, 0);
                b_k   = b_data(idx + 1, 0);
                b_k_1 = b_data(idx + 1, 1);              

                s_1 = (s_k + s_k_1 ) / 2.0 /range[idx + 1];
                velocity_s     = bezier_basis.getVel(coef, idx + 1, s_1);
                acceleration_s = bezier_basis.getAcc(coef, idx + 1, s_1) / range[idx + 1];
                acceleration2 = velocity_s * a_k + acceleration_s * (b_k + b_k_1) / 2.0;
                acceleration  = acceleration1 + (acceleration2 - acceleration1) * t / delta_t;        
            }
            else if(grid_idx == 0)// take average accleration between two segments
            {   
                int grid_num_k = K_data[idx - 1]; // last segment's grid num
                delta_t = (time(idx - 1, grid_num_k - 1) - time_acc(idx - 1, grid_num_k - 1) + time_acc(idx, 0) );
                
                s_k   = s_data(idx - 1, grid_num_k - 1);
                s_k_1 = s_data(idx - 1, grid_num_k    );
                
                a_k   = a_data(idx - 1, grid_num_k - 1);
                b_k   = b_data(idx - 1, grid_num_k - 1);
                b_k_1 = b_data(idx - 1, grid_num_k    );

                s_1 = (s_k + s_k_1 ) / 2.0 / range[idx - 1];
                velocity_s     = bezier_basis.getVel(coef, idx - 1, s_1);
                acceleration_s = bezier_basis.getAcc(coef, idx - 1, s_1) / range[idx - 1];
                acceleration1  = velocity_s * a_k + acceleration_s * (b_k + b_k_1) / 2.0;

                s_k   = s_data(idx, 0);
                s_k_1 = s_data(idx, 0 + 1);
                
                a_k   = a_data(idx, 0);
                b_k   = b_data(idx, 0);
                b_k_1 = b_data(idx, 0 + 1);

                s_1 = (s_k + s_k_1 ) / 2.0 / range[idx];
                velocity_s     = bezier_basis.getVel(coef, idx, s_1);
                acceleration_s = bezier_basis.getAcc(coef, idx, s_1) / range[idx];
                acceleration2  = velocity_s * a_k + acceleration_s * (b_k + b_k_1) / 2.0;
                acceleration   = acceleration1 + (acceleration2 - acceleration1) * (t + time(idx - 1, grid_num_k - 1) - time_acc(idx - 1, grid_num_k - 1)) / delta_t;   
            } 
        }
    }

} //namespace ft
