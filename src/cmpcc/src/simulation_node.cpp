#include <quadrotor_msgs/PositionCommand.h>
#include <geometry_msgs/Point.h>
#include <geometry_msgs/PoseStamped.h>
#include <nav_msgs/Path.h>
#include <nav_msgs/Odometry.h>
#include <sensor_msgs/Imu.h>
#include <ros/ros.h>
#include <sensor_msgs/Joy.h>
#include "mpc_solver.h"

using namespace std;
using namespace Eigen;
using namespace ft;

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

bool reset_now = false;
int fail_times = 0;

Eigen::SparseMatrix<double> stateL;
Eigen::SparseMatrix<double> stateR;
bool state_predict_done = false;

double wind_x = 0;
double wind_y = 0;

void odom_callback(const nav_msgs::Odometry& odom){
    odomRead = true;
    odomT = (odom.header.stamp - tOdom).toSec();
    tOdom = odom.header.stamp;
    pDrone << odom.pose.pose.position.x, odom.pose.pose.position.y, odom.pose.pose.position.z;
    vDrone << odom.twist.twist.linear.x, odom.twist.twist.linear.y, odom.twist.twist.linear.z;
    aDrone << odom.twist.twist.angular.x, odom.twist.twist.angular.y, odom.twist.twist.angular.z;
    stateOdomPrevious = stateOdom;
    stateOdom.coeffRef(0,0) = pDrone(0);
    stateOdom.coeffRef(3,0) = pDrone(1);
    stateOdom.coeffRef(6,0) = pDrone(2);
    stateOdom.coeffRef(1,0) = vDrone(0);
    stateOdom.coeffRef(4,0) = vDrone(1);
    stateOdom.coeffRef(7,0) = vDrone(2);
    stateOdom.coeffRef(9,0) = simSolver.map.findNearestTheta(pDrone);
    odomRead = false;
}

void mpc_callback(const ros::TimerEvent& event){
    while(ros::ok() && odomRead);
    mpcInit = true;
    tMpc = ros::Time::now();
    stateL = simSolver.statePredict.col(0);
    stateR = simSolver.statePredict.col(1);
    // calculate stateMPC
    stateMpc = (stateOdom - stateOdomPrevious)/odomT*(tMpc-tOdom).toSec() + stateOdom;
    // update some state from last horizon
    if (fail_times > 3){
        stateMpc.coeffRef( 2, 0) = 0;
        stateMpc.coeffRef( 5, 0) = 0;
        stateMpc.coeffRef( 8, 0) = 0;
        stateMpc.coeffRef(10, 0) = 0;
        stateMpc.coeffRef(11, 0) = 0;
        simSolver.initStatus = true;
        mpcInit = false;
        if( simSolver.solveMpcQp(stateMpc) ){
            ROS_ERROR("come on! Your disturbance too fierce!");
            ros::shutdown();
        }
        else{
            fail_times = 0;
        }
    }
    else{
        stateMpc.coeffRef(2, 0) = simSolver.statePredict.coeffRef(2, 1); // ax
        stateMpc.coeffRef(5, 0) = simSolver.statePredict.coeffRef(5, 1); // ay
        stateMpc.coeffRef(8, 0) = simSolver.statePredict.coeffRef(8, 1); // az
        stateMpc.coeffRef(10,0) = simSolver.statePredict.coeffRef(10,1); // vtheta
        stateMpc.coeffRef(11,0) = simSolver.statePredict.coeffRef(11,1); // atheta
        mpcInit = false;
        if( simSolver.solveMpcQp(stateMpc) ){
            fail_times ++;
        }
        else{
            fail_times = 0;
        }
    }

    state_predict_done = true;
    
    refer_pub.publish(simSolver.displayPtr->refTraj_msg);
    global_pub.publish(simSolver.displayPtr->theta_msg);
    drone_pub.publish(simSolver.displayPtr->drone_msg);
    vis_polytope_pub.publish(simSolver.displayPtr->corridor_array_msg);
    simSolver.displayPtr->pubTunnels(flight_tunnel_pub);
    predict_pub.publish(simSolver.displayPtr->trajPred_msg);
}

void cmd_callback(const ros::TimerEvent& event){
    if (state_predict_done){
        while(ros::ok() && mpcInit);
        double deltaT = (ros::Time::now() - tMpc).toSec();
        stateCmd = stateL + (stateR-stateL)*deltaT/mpcT;
        tmpPoint.x = stateCmd.coeffRef(0,0);
        tmpPoint.y = stateCmd.coeffRef(3,0);
        tmpPoint.z = stateCmd.coeffRef(6,0);
        cmdMsg.position = tmpPoint;
        tmpVector.x = stateCmd.coeffRef(1,0);
        tmpVector.y = stateCmd.coeffRef(4,0);
        tmpVector.z = stateCmd.coeffRef(7,0);
        cmdMsg.velocity = tmpVector;
        tmpVector.x = stateCmd.coeffRef(2,0) + wind_x;
        tmpVector.y = stateCmd.coeffRef(5,0) + wind_y;
        tmpVector.z = stateCmd.coeffRef(8,0);
        cmdMsg.acceleration = tmpVector;
        cmdMsg.header.stamp = ros::Time::now();
        cmd_pub.publish(cmdMsg);
    }
}

void joy_callback(const sensor_msgs::Joy::ConstPtr& msg){
    wind_x = 3 * msg->axes[0];
    wind_y = 3 * msg->axes[1];
}


int main(int argc, char **argv)
{
    ros::init(argc, argv, "simulation_node");
    ros::NodeHandle nodeHandle;
    cmd_pub = nodeHandle.advertise<quadrotor_msgs::PositionCommand>("position_cmd",1);
    refer_pub = nodeHandle.advertise<nav_msgs::Path>("refer_path", 1);
    drone_pub = nodeHandle.advertise<visualization_msgs::Marker>("drone_pose", 1);
    global_pub = nodeHandle.advertise<visualization_msgs::Marker>("global_pose", 1);
    ros::Subscriber sub_odom = nodeHandle.subscribe("odom", 5 , odom_callback, ros::TransportHints().tcpNoDelay());
    ros::Subscriber sub_joy = nodeHandle.subscribe("/joy", 5 , joy_callback, ros::TransportHints().tcpNoDelay());
    vis_polytope_pub  = nodeHandle.advertise<decomp_ros_msgs::PolyhedronArray>("polyhedron_corridor_mesh", 1, true);
    flight_tunnel_pub = nodeHandle.advertise<decomp_ros_msgs::PolyhedronArray>("flight_tunnel", 1, true);
    predict_pub = nodeHandle.advertise<nav_msgs::Path>("predict_path", 1);
    globalOdom_pub = nodeHandle.advertise<nav_msgs::Odometry>("globalOdom", 1);

    ros::Timer timer_mpc = nodeHandle.createTimer(ros::Duration(mpcT), mpc_callback);
    ros::Timer timer_cmd = nodeHandle.createTimer(ros::Duration(0.01), cmd_callback);

    ros::MultiThreadedSpinner spinner(4);

    cmdMsg.header.frame_id = "world";
    
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
    ros::Rate loopRate(10);
    ros::Time startT = ros::Time::now();
    while(ros::ok()){
        ros::Time nowT = ros::Time::now();
        cmd_pub.publish(cmdMsg);
        if ((nowT-startT).toSec() > 3){
            break;
        }
        loopRate.sleep();
    }

    spinner.spin();
    return 0;
}
