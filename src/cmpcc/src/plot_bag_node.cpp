#include <quadrotor_msgs/PositionCommand.h>
#include <geometry_msgs/Point.h>
#include <geometry_msgs/PoseStamped.h>
#include <nav_msgs/Path.h>
#include <nav_msgs/Odometry.h>
#include <sensor_msgs/Imu.h>
#include <ros/ros.h>
#include "mpc_solver.h"

using namespace std;
using namespace ft;
using namespace Eigen;

ft::MpcSolver simSolver;
Matrix3d w_R_odom;
Vector3d w_t_odom;

double start_time;

void rcvTransformCallBack(geometry_msgs::PoseConstPtr msg){
    Quaterniond q(msg->orientation.w, msg->orientation.x, msg->orientation.y, msg->orientation.z);
    w_R_odom = q.toRotationMatrix();
    w_t_odom << msg->position.x, msg->position.y, msg->position.z;
}

void odom_callback(const nav_msgs::Odometry& odom){
    double t = odom.header.stamp.toSec() - start_time;
    if (t < 0){
        return;
    }
    Vector3d pDrone( odom.pose.pose.position.x, odom.pose.pose.position.y, odom.pose.pose.position.z);
    Vector3d vDrone( odom.twist.twist.linear.x, odom.twist.twist.linear.y, odom.twist.twist.linear.z);
    Vector3d aDrone( odom.twist.twist.angular.x, odom.twist.twist.angular.y, odom.twist.twist.angular.z);
    pDrone= w_R_odom * pDrone + w_t_odom;
    vDrone = w_R_odom * vDrone;
    aDrone = w_R_odom * aDrone;
    double theta = simSolver.map.findNearestTheta(pDrone);
    Vector3d p, v, a;
    simSolver.map.getGlobalCommand(t, p, v, a);
    cout << t << " "
         << pDrone.transpose() << " "
         << vDrone.transpose() << " "
         << aDrone.transpose() << " "
         << p.transpose() << " "         
         << v.transpose() << " "
         << a.transpose() << " "
         << endl;
}

void cmd_callback(const quadrotor_msgs::PositionCommand& cmd){
    double t = cmd.header.stamp.toSec() - start_time;
    if (t < 0){
        return;
    }
    Vector3d pos, vel, acc;
    pos << cmd.position.x, cmd.position.y, cmd.position.z;
    double theta = simSolver.map.findNearestTheta(pos);
    simSolver.map.getGlobalCommand(theta, pos, vel, acc);
    pos = w_R_odom.transpose() * (pos - w_t_odom);
    vel = w_R_odom.transpose() * vel;
    acc = w_R_odom.transpose() * acc;
    cout << t << " " 
         << cmd.position.x << " "
         << cmd.position.y << " "
         << cmd.position.z << " "
         << cmd.velocity.x << " "
         << cmd.velocity.y << " "
         << cmd.velocity.z << " "
         << cmd.acceleration.x << " "
         << cmd.acceleration.y << " "
         << cmd.acceleration.z << " "
         << pos.transpose() << " "
         << vel.transpose() << " "
         << acc.transpose() << " "
         << endl;
}

int main(int argc, char **argv){
    ros::init(argc, argv, "plot_bag_node");
    ros::NodeHandle n("~");
    n.getParam("start_time", start_time);
    ros::Subscriber sub_odom = n.subscribe("/vins_estimator/imu_propagate", 5 , odom_callback, ros::TransportHints().tcpNoDelay());
    // ros::Subscriber sub_odom = n.subscribe("drone_1/state_ukf/odom", 5 , odom_callback, ros::TransportHints().tcpNoDelay());
    // ros::Subscriber sub_cmd = n.subscribe("/position_cmd", 5 , cmd_callback, ros::TransportHints().tcpNoDelay());
    // ros::Subscriber sub_cmd = n.subscribe("drone_1/position_cmd", 5 , cmd_callback, ros::TransportHints().tcpNoDelay());
    ros::Subscriber transformation_sub = n.subscribe("/loop_fusion/pg_T_vio", 1, rcvTransformCallBack, ros::TransportHints().tcpNoDelay());
    w_R_odom = Matrix3d::Identity();
    w_t_odom = Vector3d::Zero();
    ros::spin();
    return 0;
}



