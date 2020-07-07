#include <ros/ros.h>
#include <iostream>
#include "plan_env/sdf_map.h"
#include "plan_env/raycast.h"


SDFMap::Ptr sdf_map_;

int main(int argc, char **argv)
{
    ros::init(argc, argv, "plan_env_node");
    ros::NodeHandle nodeHandle("~");

    sdf_map_.reset(new SDFMap);
    sdf_map_->initMap(nodeHandle);



    ros::spin();
    return 0;

}