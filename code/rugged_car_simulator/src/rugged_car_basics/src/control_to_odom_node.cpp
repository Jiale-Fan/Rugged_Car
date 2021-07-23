#include <ros/ros.h>

#include "rugged_car_basics/control_to_odom.h"

int main(int argc, char** argv)
{
  ros::init(argc, argv, "control_to_odom_node");
  ros::NodeHandle nh;
  ros::NodeHandle private_nh("~");

  controlToOdom control_to_odom(nh, private_nh);

  ros::spin();

  return 0;
}
