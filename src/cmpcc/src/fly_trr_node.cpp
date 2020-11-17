#include <quadrotor_msgs/PositionCommand.h>
#include <geometry_msgs/Point.h>
#include <geometry_msgs/PoseStamped.h>
#include <nav_msgs/Path.h>
#include <nav_msgs/Odometry.h>
#include <sensor_msgs/Imu.h>
#include <ros/ros.h>
#include "mpc_solver.h"

using namespace std;
using namespace Eigen;
using namespace ft;
ros::Time nowT;
int fail_times = 0;

ros::Publisher cmd_pub, refer_pub, drone_pub, global_pub, vis_polytope_pub, flight_tunnel_pub, predict_pub, globalOdom_pub;

ft::MpcSolver simSolver;
quadrotor_msgs::PositionCommand cmdMsg;
nav_msgs::Odometry globalOdom;
// Eigen::SparseMatrix<double,Eigen::RowMajor> stateOdom(Model::numState,1);
Eigen::SparseMatrix<double> stateCmd(Model::numState,1);
Eigen::SparseMatrix<double> stateMpc(Model::numState,1);
Eigen::SparseMatrix<double> stateOdom(Model::numState,1);
Eigen::SparseMatrix<double> stateOdomPrevious(Model::numState,1);
double mpcT = 0.05;
double odomT;
Eigen::Vector3d pDrone, vDrone, aDrone;
bool mpcInit, odomRead, isGlobal, getDestination;
geometry_msgs::Point tmpPoint;
geometry_msgs::Vector3 tmpVector;
ros::Time tStart, tFinal, tOdom, tMpc;

// vio
Matrix3d w_R_odom;
Vector3d w_t_odom;
bool update_transform = false;
bool update_mpc_tr = false;

int mycount = 0;

// vio loop
void rcvTransformCallBack(geometry_msgs::PoseConstPtr msg){
    Quaterniond q(msg->orientation.w, msg->orientation.x, msg->orientation.y, msg->orientation.z);

    w_R_odom = q.toRotationMatrix();
    
    w_t_odom << msg->position.x, msg->position.y, msg->position.z;
    update_transform = true;
}

void odom_callback(const nav_msgs::Odometry& odom){
    double t = (ros::Time::now() - nowT).toSec();
    Vector3d pos, vel ,acc;
    if(t < simSolver.map.thetaMax){
    simSolver.map.getGlobalCommand(t, pos, vel, acc);
    pos = w_R_odom.transpose() * (pos - w_t_odom);
    vel = w_R_odom.transpose() * vel;
    acc = w_R_odom.transpose() * acc;
    cmdMsg.header.frame_id = "world";
    cmdMsg.header.seq = mycount++;
    cmdMsg.header.stamp = ros::Time::now();

    tmpPoint.x = pos(0);
    tmpPoint.y = pos(1);
    tmpPoint.z = pos(2);
    cmdMsg.position = tmpPoint;
    tmpVector.x = vel(0);
    tmpVector.y = vel(1);
    tmpVector.z = vel(2);
    cmdMsg.velocity = tmpVector;
    tmpVector.x = acc(0);
    tmpVector.y = acc(1);
    tmpVector.z = acc(2);
    cmdMsg.acceleration = tmpVector;
    cmdMsg.trajectory_flag = 1u;
    cmdMsg.yaw = atan2(cmdMsg.velocity.y, cmdMsg.velocity.x);
    cmdMsg.yaw_dot = 1.0;
    }
    else{
        pos << odom.pose.pose.position.x, odom.pose.pose.position.y, odom.pose.pose.position.z;
        vel << 0,0,0;
        acc << 0,0,0;
        tmpPoint.x = pos(0);
        tmpPoint.y = pos(1);
        tmpPoint.z = pos(2);
        cmdMsg.position = tmpPoint;
        tmpVector.x = vel(0);
        tmpVector.y = vel(1);
        tmpVector.z = vel(2);
        cmdMsg.velocity = tmpVector;
        tmpVector.x = acc(0);
        tmpVector.y = acc(1);
        tmpVector.z = acc(2);
        cmdMsg.acceleration = tmpVector;
        cmdMsg.trajectory_flag = 1u;
        cmdMsg.yaw = 0;
        cmdMsg.yaw_dot = 1.0;
    }
    cmdMsg.acceleration.x += 2;
    cmd_pub.publish(cmdMsg);
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "fly_trr_node");
    ros::NodeHandle nodeHandle;
    cmd_pub = nodeHandle.advertise<quadrotor_msgs::PositionCommand>("drone_1/position_cmd",10);
    refer_pub = nodeHandle.advertise<nav_msgs::Path>("refer_path", 1000);
    drone_pub = nodeHandle.advertise<visualization_msgs::Marker>("drone_pose", 1000);
    global_pub = nodeHandle.advertise<visualization_msgs::Marker>("global_pose", 1000);
    ros::Subscriber sub_odom = nodeHandle.subscribe("drone_1/state_ukf/odom", 5 , odom_callback, ros::TransportHints().tcpNoDelay());
    // ros::Subscriber sub_odom = nodeHandle.subscribe("/drone_1/visual_slam/odom", 5 , odom_callback);
    vis_polytope_pub  = nodeHandle.advertise<decomp_ros_msgs::PolyhedronArray>("polyhedron_corridor_mesh", 1000, true);
    flight_tunnel_pub = nodeHandle.advertise<decomp_ros_msgs::PolyhedronArray>("flight_tunnel", 1000, true);
    predict_pub = nodeHandle.advertise<nav_msgs::Path>("predict_path", 1000);
    globalOdom_pub = nodeHandle.advertise<nav_msgs::Odometry>("globalOdom", 1000);

    ros::MultiThreadedSpinner spinner(4);

    cmdMsg.header.frame_id = "world";
    cmdMsg.header.seq = mycount++;
    cmdMsg.header.stamp = ros::Time::now();

    // vio
    w_R_odom = Matrix3d::Identity();
    w_t_odom = Vector3d::Zero();
    
    // init position: 
    simSolver.map.getGlobalCommand(0, pDrone);
    tmpPoint.x = pDrone(0);
    tmpPoint.y = pDrone(1);
    tmpPoint.z = pDrone(2);
    cmdMsg.position = tmpPoint;
    tmpVector.x = 0;
    tmpVector.y = 0;
    tmpVector.z = 0;
    cmdMsg.velocity = tmpVector;
    tmpVector.x = 0;
    tmpVector.y = 0;
    tmpVector.z = 0;
    cmdMsg.acceleration = tmpVector;
    cmdMsg.trajectory_flag = 1u;
    cmdMsg.yaw = 0;
    cmdMsg.yaw_dot = 0.0;
    ros::Rate loopRate(10);
    ros::Time startT = ros::Time::now();
    while(ros::ok()){
        nowT = ros::Time::now();
        cmd_pub.publish(cmdMsg);
        if ((nowT-startT).toSec() > 5){
            cout << "start_time_stamp: " << nowT << endl;
            break;
        }
        loopRate.sleep();
    }

    spinner.spin();
    return 0;
}
