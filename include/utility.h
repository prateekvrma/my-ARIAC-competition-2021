#ifndef UTILITY_H
#define UTILITY_H

#include <cmath>
#include <tuple>

#include <nist_gear/Model.h>
#include <nist_gear/Product.h>
#include <geometry_msgs/Quaternion.h>
#include <geometry_msgs/Pose.h>

namespace Utility 
{
  bool is_same_part(const nist_gear::Model& part1, const nist_gear::Model& part2, double tolerance); 

  void print_part_pose(const nist_gear::Model& part); 
  void print_part_pose(const nist_gear::Product& part); 
  void print_pose(const geometry_msgs::Pose& pose); 

  std::tuple<double, double, double> quat_to_rpy(const geometry_msgs::Quaternion& quat); 
}

#endif 


