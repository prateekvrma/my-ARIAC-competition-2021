#ifndef UTILITY_H
#define UTILITY_H

#include <cmath>
#include <tuple>
#include <string>
#include <cstdint>
#include <array>

#include <ros/ros.h>

#include <nist_gear/Model.h>
#include <nist_gear/Product.h>
#include <geometry_msgs/Quaternion.h>
#include <geometry_msgs/Pose.h>
#include <tf2/LinearMath/Quaternion.h>

namespace Utility 
{
  bool is_same_part(const nist_gear::Model& part1, const nist_gear::Model& part2, double tolerance); 

  void print_part_pose(const nist_gear::Model& part); 
  void print_part_pose(const nist_gear::Product& part); 
  void print_pose(const geometry_msgs::Pose& pose); 

  std::tuple<double, double, double> quat_to_rpy(const geometry_msgs::Quaternion& quat); 

  namespace motioncontrol {
    geometry_msgs::Pose transformToWorldFrame(const geometry_msgs::Pose& target,std::string agv);
    geometry_msgs::Pose transformToWorldFrame(std::string part_in_camera_frame);
    geometry_msgs::Pose transformBriefcaseToWorldFrame(const geometry_msgs::Pose& target, std::string station); 

    geometry_msgs::Pose transformToTrayFrame(const geometry_msgs::Pose& target,std::string agv); 
    std::array<double, 3> eulerFromQuaternion(const geometry_msgs::Pose& pose);
    std::array<double, 3> eulerFromQuaternion(double x, double y, double z, double w);
    std::array<double, 3> eulerFromQuaternion(const tf2::Quaternion& quat);
    tf2::Quaternion quaternionFromEuler(double r, double p, double y);
    template <typename T>
    bool contains(std::vector<T> vec, const T& elem);

    /**
     * @brief Print the components of a quaternion
     * 
     * @param quat tf2::Quaternion to print
     */
    void print(const tf2::Quaternion& quat);
    void print(const geometry_msgs::Pose& pose);
  }

  namespace location {
    std::string get_pose_location(const geometry_msgs::Pose& pose);  

    // part location
    //     0 1
    //     2 3
    // conveyer belt
    int get_pose_location_in_bin(const geometry_msgs::Pose& pose, const std::string& bin_id);  
    geometry_msgs::Pose get_pose_from_bin_location(const std::string& bin_id, int i);  
  }
}

#endif 


