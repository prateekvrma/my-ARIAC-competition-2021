#include "utility.h"

#include <ros/ros.h>

namespace Utility
{
  bool is_same_part(const nist_gear::Model& part1, const nist_gear::Model& part2)
  {
    double epsilon = 0.05; 

    auto x1 = part1.pose.position.x; 
    auto y1 = part1.pose.position.y; 
    auto z1 = part1.pose.position.z; 

    auto x2 = part2.pose.position.x; 
    auto y2 = part2.pose.position.y; 
    auto z2 = part2.pose.position.z; 

    // auto dist = sqrt(pow((x2-x1), 2) + pow((y2-y1), 2) + pow((z2-z1), 2)); 
    // ROS_INFO("distance: %f", dist); 

    if (sqrt(pow((x2-x1), 2) + pow((y2-y1), 2) + pow((z2-z1), 2)) < epsilon)
      return true; 
    else
      return false; 
  }

  void print_part_pose(const nist_gear::Model& part)
  {
    ROS_INFO("%s in /world frame: [%f,%f,%f]",
              part.type.c_str(), 
              part.pose.position.x,
              part.pose.position.y,
              part.pose.position.z); 

  }
}
