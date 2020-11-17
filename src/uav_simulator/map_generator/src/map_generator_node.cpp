#include <iostream>
#include <pcl/io/pcd_io.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>

#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <ros/package.h>

int main (int argc, char** argv) 
{        
   ros::init (argc, argv, "map_generator_node");
   ros::NodeHandle n( "~" );

   double sense_rate;
   std::string map_file_name;
   n.param("sensing/rate", sense_rate, 1.0);
   n.param("map/file_name", map_file_name, std::string(""));
   std::string map_path = ros::package::getPath("map_generator")+"/config/"+map_file_name.c_str();

   ros::Publisher map_pub = n.advertise<sensor_msgs::PointCloud2>("global_map", 1);

   sensor_msgs::PointCloud2 global_cloud_msg;
   pcl::PointCloud<pcl::PointXYZ> global_cloud;
   if (pcl::io::loadPCDFile<pcl::PointXYZ>(map_path, global_cloud) == 0 ){
      pcl::toROSMsg(global_cloud, global_cloud_msg);
      global_cloud_msg.header.frame_id = "world";
   }

   ros::Rate loop_rate(sense_rate);
   while (ros::ok())
   {
      map_pub.publish(global_cloud_msg);
      ros::spinOnce();
      loop_rate.sleep();
   }
}