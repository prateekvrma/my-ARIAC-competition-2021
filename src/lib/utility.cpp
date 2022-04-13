#include "utility.h"

#include <ros/ros.h>

#include <tf2_ros/transform_listener.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h> //--needed for tf2::Matrix3x3


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
    double roll, pitch, yaw;
    std::tie(roll, pitch, yaw) = Utility::quat_to_rpy(part.pose.orientation); 

    ROS_INFO("%s in /world frame: [%f,%f,%f], [%f,%f,%f]",
              part.type.c_str(), 
              part.pose.position.x,
              part.pose.position.y,
              part.pose.position.z,
              roll,
              pitch,
              yaw); 

  }

  void print_part_pose(const nist_gear::Product& part)
  {
    double roll, pitch, yaw;
    std::tie(roll, pitch, yaw) = Utility::quat_to_rpy(part.pose.orientation); 

    ROS_INFO("%s in /world frame: [%f,%f,%f], [%f,%f,%f]",
              part.type.c_str(), 
              part.pose.position.x,
              part.pose.position.y,
              part.pose.position.z,
              roll,
              pitch,
              yaw); 

  }

  std::tuple<double, double, double> quat_to_rpy(const geometry_msgs::Quaternion& quat)
  {
      // Transform msgs quaternion to tf2 quaternion 
      tf2::Quaternion q_tf;
      tf2::convert(quat, q_tf); 
      q_tf.normalize(); 

      //Convert Quaternion to Euler angles
      tf2::Matrix3x3 m(q_tf);
      double roll, pitch, yaw;
      m.getRPY(roll, pitch, yaw);

      return std::make_tuple(roll, pitch, yaw); 
    }
}
