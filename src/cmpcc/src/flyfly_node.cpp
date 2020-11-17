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


// vio loop
void rcvTransformCallBack(geometry_msgs::PoseConstPtr msg){
    Quaterniond q(msg->orientation.w, msg->orientation.x, msg->orientation.y, msg->orientation.z);

    w_R_odom = q.toRotationMatrix();
    
    w_t_odom << msg->position.x, msg->position.y, msg->position.z;
    update_transform = true;
}

void odom_callback(const nav_msgs::Odometry& odom){
    odomRead = true;
    odomT = (odom.header.stamp - tOdom).toSec();
    tOdom = odom.header.stamp;
    pDrone << odom.pose.pose.position.x, odom.pose.pose.position.y, odom.pose.pose.position.z;
    vDrone << odom.twist.twist.linear.x, odom.twist.twist.linear.y, odom.twist.twist.linear.z;
    aDrone << odom.twist.twist.angular.x, odom.twist.twist.angular.y, odom.twist.twist.angular.z;
    stateOdom.coeffRef(9,0) = simSolver.map.findNearestTheta(pDrone);
    odomRead = false;
}

void mpc_callback(const ros::TimerEvent& event){
    while(ros::ok() && odomRead);
    mpcInit = true;
    tMpc = ros::Time::now();
    // calculate stateMPC
    // stateMpc = (stateOdom - stateOdomPrevious)/odomT*(tMpc-tOdom).toSec() + stateOdom;
    update_mpc_tr = false;
    if (update_transform){
        update_mpc_tr = true;
        Eigen::Vector3d p = w_R_odom * pDrone + w_t_odom;
        Eigen::Vector3d v = w_R_odom * vDrone;
        stateMpc.coeffRef(0,0) = p(0);
        stateMpc.coeffRef(3,0) = p(1);
        stateMpc.coeffRef(6,0) = p(2);
        stateMpc.coeffRef(1,0) = v(0);
        stateMpc.coeffRef(4,0) = v(1);
        stateMpc.coeffRef(7,0) = v(2);
    }
    else{
        stateMpc.coeffRef(0,0) = pDrone(0);
        stateMpc.coeffRef(3,0) = pDrone(1);
        stateMpc.coeffRef(6,0) = pDrone(2);
        stateMpc.coeffRef(1,0) = vDrone(0);
        stateMpc.coeffRef(4,0) = vDrone(1);
        stateMpc.coeffRef(7,0) = vDrone(2);
    }
    update_transform = false;
    // integral
    stateMpc.coeffRef(2, 0) = simSolver.statePredict.coeffRef(2, 1); // ax
    stateMpc.coeffRef(5, 0) = simSolver.statePredict.coeffRef(5, 1); // ay
    stateMpc.coeffRef(8, 0) = simSolver.statePredict.coeffRef(8, 1); // az
    stateMpc.coeffRef(10,0) = simSolver.statePredict.coeffRef(10,1); // vtheta
    stateMpc.coeffRef(11,0) = simSolver.statePredict.coeffRef(11,1); // atheta
    mpcInit = false;
    simSolver.solveMpcQp(stateMpc);
    
    refer_pub.publish(simSolver.displayPtr->refTraj_msg);
    global_pub.publish(simSolver.displayPtr->theta_msg);
    drone_pub.publish(simSolver.displayPtr->drone_msg);
    vis_polytope_pub.publish(simSolver.displayPtr->corridor_array_msg);
    simSolver.displayPtr->pubTunnels(flight_tunnel_pub);
    predict_pub.publish(simSolver.displayPtr->trajPred_msg);
}

void cmd_callback(const ros::TimerEvent& event){
    while(ros::ok() && mpcInit);
    double deltaT = (ros::Time::now() - tMpc).toSec();
    Eigen::SparseMatrix<double> stateL = simSolver.statePredict.col(0);
    Eigen::SparseMatrix<double> stateR = simSolver.statePredict.col(1);
    stateCmd = stateL + (stateR-stateL)*deltaT/mpcT;
    Eigen::Vector3d pos(stateCmd.coeffRef(0,0), stateCmd.coeffRef(3,0), stateCmd.coeffRef(6,0));
    Eigen::Vector3d vel(stateCmd.coeffRef(1,0), stateCmd.coeffRef(4,0), stateCmd.coeffRef(7,0));
    Eigen::Vector3d acc(stateCmd.coeffRef(2,0), stateCmd.coeffRef(5,0), stateCmd.coeffRef(8,0));
    // vio {
    if (update_mpc_tr){
        pos = w_R_odom.transpose() * (pos - w_t_odom);
        vel = w_R_odom.transpose() * vel;
        acc = w_R_odom.transpose() * acc;
    }
    // } vio
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
    cmd_pub.publish(cmdMsg);
}

void safe_callback(const ros::TimerEvent& event){

}


int main(int argc, char **argv)
{
    ros::init(argc, argv, "flyfly_node");
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
    // vio
    ros::Subscriber transformation_sub = nodeHandle.subscribe("w_T_odom", 1, rcvTransformCallBack, ros::TransportHints().tcpNoDelay());

    ros::Timer timer_mpc = nodeHandle.createTimer(ros::Duration(mpcT), mpc_callback);
    ros::Timer timer_cmd = nodeHandle.createTimer(ros::Duration(0.01), cmd_callback);
    // ros::Timer timer_safe = nodeHandle.createTimer(ros::Duration(0.005), safe_callback);

    ros::MultiThreadedSpinner spinner(4);

    cmdMsg.header.frame_id = "world";

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
    ros::Rate loopRate(10);
    ros::Time startT = ros::Time::now();
    while(ros::ok()){
        ros::Time nowT = ros::Time::now();
        cmd_pub.publish(cmdMsg);
        if ((nowT-startT).toSec() > 5){
            break;
        }
        loopRate.sleep();
    }

    spinner.spin();
    return 0;
}
