#include <iostream>
#include <pcl/io/pcd_io.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/search/kdtree.h>
#include <pcl/search/impl/kdtree.hpp>

#include <ros/ros.h>
#include <ros/console.h>
#include <sensor_msgs/PointCloud2.h>
#include <geometry_msgs/Vector3.h>
#include <geometry_msgs/PoseStamped.h>
#include <nav_msgs/Odometry.h>
#include <Eigen/Eigen>
#include <math.h>
#include <ros/package.h>

using namespace std;
using namespace Eigen;

ros::Publisher _all_map_pub;
ros::Subscriber _odom_sub;

int _obs_num, _cir_num;
double _x_size, _y_size, _z_size, _init_x, _init_y, _resolution, _sense_rate;
double _x_l, _x_h, _y_l, _y_h, _w_l, _w_h, _h_l, _h_h, _w_c_l, _w_c_h;
std::string _map_file_name;

bool _has_map  = false;
bool _has_odom = false;

sensor_msgs::PointCloud2 globalMap_pcd;
pcl::PointCloud<pcl::PointXYZ> cloudMap, cloudMapCut;

pcl::search::KdTree<pcl::PointXYZ> kdtreeMap;
vector<int>     pointIdxSearch;
vector<float>   pointSquaredDistance;      

void rcvOdometryCallbck(const nav_msgs::Odometry odom)
{
   _has_odom = true;
}

void pubSensedPoints()
{     
   //if(!_has_map || !_has_odom)
   if( !_has_map ) return;

   _all_map_pub.publish(globalMap_pcd);
}

int main (int argc, char** argv) 
{        
   ros::init (argc, argv, "generate_map_node");
   ros::NodeHandle n( "~" );

   _all_map_pub   = n.advertise<sensor_msgs::PointCloud2>("global_map", 1);                      
   _odom_sub      = n.subscribe( "odometry", 50, rcvOdometryCallbck);

   n.param("init_state_x", _init_x, 0.0);
   n.param("init_state_y", _init_y, 0.0);

   n.param("map/x_size",  _x_size, 50.0);
   n.param("map/y_size",  _y_size, 50.0);
   n.param("map/z_size",  _z_size, 5.0 );

   n.param("map/obs_num",    _obs_num,  30);
   n.param("map/circle_num", _cir_num,  30);
   n.param("map/resolution", _resolution, 0.2);

   n.param("ObstacleShape/lower_rad", _w_l,   0.3);
   n.param("ObstacleShape/upper_rad", _w_h,   0.8);
   n.param("ObstacleShape/lower_hei", _h_l,   3.0);
   n.param("ObstacleShape/upper_hei", _h_h,   7.0);

   n.param("CircleShape/lower_circle_rad", _w_c_l, 0.3);
   n.param("CircleShape/upper_circle_rad", _w_c_h, 0.8);

   n.param("sensing/rate", _sense_rate, 1.0);

   n.param("map/file_name", _map_file_name, string(""));

   _x_l = - _x_size / 2.0;
   _x_h = + _x_size / 2.0;

   _y_l = - _y_size / 2.0;
   _y_h = + _y_size / 2.0;


   //RandomMapGenerate();

   //zxzxzxzx
   std::string path = ros::package::getPath("environment")+"/config/"+_map_file_name.c_str();
   if (pcl::io::loadPCDFile<pcl::PointXYZ>(path, cloudMap) == 0 ){
      cout << "cloudMap.size()" << cloudMap.size() << endl;
      _has_map = true;
      for ( int i=0; i<cloudMap.points.size(); i++ )
      {
         if ( cloudMap.points[i].z < 2.0 && cloudMap.points[i].y > -8.1  && cloudMap.points[i].x < 7.6 )
         {
            if ( cloudMap.points[i].z < 2.0 && cloudMap.points[i].y < -6  && cloudMap.points[i].x < 1 )
               continue;
            if ( cloudMap.points[i].z < 2.0 && cloudMap.points[i].y > -4  && cloudMap.points[i].x > 7.2 )
               continue;
            cloudMapCut.points.push_back( cloudMap.points[i] );
         }
      }
      pcl::toROSMsg(cloudMapCut, globalMap_pcd);
      globalMap_pcd.header.frame_id = "map";
    //   ROS_WARN("read PCD from ~/.ros/");

   }

   ros::Rate loop_rate(_sense_rate);
   while (ros::ok())
   {
      pubSensedPoints();
      ros::spinOnce();
      loop_rate.sleep();
   }
}