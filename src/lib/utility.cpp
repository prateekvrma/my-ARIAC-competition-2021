#include "utility.h"

#include <ros/ros.h>

#include <tf2_ros/transform_listener.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h> //--needed for tf2::Matrix3x3
#include <geometry_msgs/TransformStamped.h>
#include <tf2_ros/transform_broadcaster.h>
#include <tf2_ros/static_transform_broadcaster.h>
#include <tf2_ros/transform_listener.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Matrix3x3.h>
#include <array>

namespace Utility
{
  double distance(const nist_gear::Model& part1, const nist_gear::Model& part2) {
    auto x1 = part1.pose.position.x; 
    auto y1 = part1.pose.position.y; 

    auto x2 = part2.pose.position.x; 
    auto y2 = part2.pose.position.y; 

    return sqrt(pow((x2-x1), 2) + pow((y2-y1), 2)); 
  }

  double angle_distance(const nist_gear::Model& part1, const nist_gear::Model& part2, std::string rpy) {
    auto part1_rpy = Utility::motioncontrol::eulerFromQuaternion(part1.pose);
    auto part2_rpy = Utility::motioncontrol::eulerFromQuaternion(part2.pose);

    if (rpy == "roll") {
        auto r1 = part1_rpy.at(0); 
        auto r2 = part2_rpy.at(0); 
        if (r1 < 0) {
           r1 = 2 * M_PI + r1; 
        }
        if (r2 < 0) {
           r2 = 2 * M_PI + r2; 
        }
        return std::min(abs(r1 - r2), 2 * M_PI - abs(r1 - r2));  
    }
    else if (rpy == "pitch") {
        auto p1 = part1_rpy.at(1); 
        auto p2 = part2_rpy.at(1); 
        if (p1 < 0) {
           p1 = 2 * M_PI + p1; 
        }
        if (p2 < 0) {
           p2 = 2 * M_PI + p2; 
        }
        return std::min(abs(p1 - p2), 2 * M_PI - abs(p1 - p2));  
    }
    else if (rpy == "yaw") {
        auto y1 = part1_rpy.at(2); 
        auto y2 = part2_rpy.at(2); 
        if (y1 < 0) {
           y1 = 2 * M_PI + y1; 
        }
        if (y2 < 0) {
           y2 = 2 * M_PI + y2; 
        }
        return std::min(abs(y1 - y2), 2 * M_PI - abs(y1 - y2));  
    }
  }

  bool is_same_part(const nist_gear::Model& part1, const nist_gear::Model& part2, double tolerance = 0.05)
  {
    if (distance(part1, part2) < tolerance)
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

  void print_pose(const geometry_msgs::Pose& pose)
  {
    ROS_INFO("Pose in /world frame: [%f,%f,%f], [%f,%f,%f,%f]",
              pose.position.x,
              pose.position.y,
              pose.position.z,
              pose.orientation.x,
              pose.orientation.y,
              pose.orientation.z,
              pose.orientation.w); 
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

  namespace motioncontrol
  {

    void print(const tf2::Quaternion& quat) {
        ROS_INFO("[x: %f, y: %f, z: %f, w: %f]",
            quat.getX(), quat.getY(), quat.getZ(), quat.getW());
    }

    void print(const geometry_msgs::Pose& pose) {
        auto rpy = eulerFromQuaternion(pose);

        ROS_INFO("position: [x: %f, y: %f, z: %f]\norientation(quat): [x: %f, y: %f, z: %f, w: %f]\norientation(rpy): [roll: %f, pitch: %f, yaw: %f]",
            pose.position.x, pose.position.y, pose.position.z,
            pose.orientation.x, pose.orientation.y, pose.orientation.z, pose.orientation.w,
            rpy[0], rpy[1], rpy[2]);
    }

    tf2::Quaternion quaternionFromEuler(double r, double p, double y) {
        tf2::Quaternion q;
        q.setRPY(r, p, y);
        // ROS_INFO("quat: [%f, %f, %f, %f]",
        //     q.getX(),
        //     q.getY(),
        //     q.getZ(),
        //     q.getW());

        return q;
    }

    std::array<double, 3> eulerFromQuaternion(const tf2::Quaternion& quat) {

        tf2::Quaternion q(
            quat.getX(),
            quat.getY(),
            quat.getZ(),
            quat.getW());
        tf2::Matrix3x3 m(q);


        double roll, pitch, yaw;
        m.getRPY(roll, pitch, yaw);

        ROS_INFO("[%f, %f, %f]", roll, pitch, yaw);

        std::array<double, 3> rpy_array{ roll, pitch, yaw };
        return rpy_array;
    }

    std::array<double, 3> eulerFromQuaternion(
        const geometry_msgs::Pose& pose) {
        tf2::Quaternion q(
            pose.orientation.x,
            pose.orientation.y,
            pose.orientation.z,
            pose.orientation.w);
        tf2::Matrix3x3 m(q);


        double roll, pitch, yaw;
        m.getRPY(roll, pitch, yaw);
        ROS_INFO("[%f, %f, %f]", roll, pitch, yaw);

        std::array<double, 3> rpy_array{ roll, pitch, yaw };
        return rpy_array;
    }

    std::array<double, 3> eulerFromQuaternion(
        double x, double y, double z, double w) {
        tf2::Quaternion q(x, y, z, w);
        tf2::Matrix3x3 m(q);


        double roll, pitch, yaw;
        m.getRPY(roll, pitch, yaw);

        ROS_INFO("%f, %f, %f", roll, pitch, yaw);

        std::array<double, 3> rpy_array{ roll, pitch, yaw };
        return rpy_array;
    }

    geometry_msgs::Pose transformToWorldFrame(std::string part_in_camera_frame) {
        tf2_ros::Buffer tfBuffer;
        tf2_ros::TransformListener tfListener(tfBuffer);
        ros::Rate rate(10);
        ros::Duration timeout(1.0);


        geometry_msgs::TransformStamped world_target_tf;
        geometry_msgs::TransformStamped ee_target_tf;


        for (int i = 0; i < 10; i++) {
            try {
                world_target_tf = tfBuffer.lookupTransform("world", part_in_camera_frame,
                    ros::Time(0), timeout);
            }
            catch (tf2::TransformException& ex) {
                ROS_WARN("%s", ex.what());
                ros::Duration(1.0).sleep();
                continue;
            }
        }

        geometry_msgs::Pose world_target{};
        world_target.position.x = world_target_tf.transform.translation.x;
        world_target.position.y = world_target_tf.transform.translation.y;
        world_target.position.z = world_target_tf.transform.translation.z;
        world_target.orientation.x = world_target_tf.transform.rotation.x;
        world_target.orientation.y = world_target_tf.transform.rotation.y;
        world_target.orientation.z = world_target_tf.transform.rotation.z;
        world_target.orientation.w = world_target_tf.transform.rotation.w;

        return world_target;
    }
    
    geometry_msgs::Pose transformBriefcaseToWorldFrame(
        const geometry_msgs::Pose& target,
        std::string station) {
        static tf2_ros::StaticTransformBroadcaster br;
        geometry_msgs::TransformStamped transformStamped;

        std::string briefcase;
        if (station.compare("as1") == 0)
            briefcase = "briefcase_1";
        else if (station.compare("as2") == 0)
            briefcase = "briefcase_2";
        else if (station.compare("as3") == 0)
            briefcase = "briefcase_3";
        else if (station.compare("as4") == 0)
            briefcase = "briefcase_4";

        transformStamped.header.stamp = ros::Time::now();
        transformStamped.header.frame_id = briefcase;
        transformStamped.child_frame_id = "target_frame_briefcase";
        transformStamped.transform.translation.x = target.position.x;
        transformStamped.transform.translation.y = target.position.y;
        transformStamped.transform.translation.z = target.position.z;
        transformStamped.transform.rotation.x = target.orientation.x;
        transformStamped.transform.rotation.y = target.orientation.y;
        transformStamped.transform.rotation.z = target.orientation.z;
        transformStamped.transform.rotation.w = target.orientation.w;


        for (int i{ 0 }; i < 15; ++i)
            br.sendTransform(transformStamped);

        tf2_ros::Buffer tfBuffer;
        tf2_ros::TransformListener tfListener(tfBuffer);
        ros::Rate rate(10);
        ros::Duration timeout(1.0);


        geometry_msgs::TransformStamped world_pose_tf;
        geometry_msgs::TransformStamped ee_target_tf;


        for (int i = 0; i < 10; i++) {
            try {
                world_pose_tf = tfBuffer.lookupTransform("world", "target_frame_briefcase",
                    ros::Time(0), timeout);
            }
            catch (tf2::TransformException& ex) {
                ROS_WARN("%s", ex.what());
                ros::Duration(1.0).sleep();
                continue;
            }
        }

        geometry_msgs::Pose world_pose{};
        world_pose.position.x = world_pose_tf.transform.translation.x;
        world_pose.position.y = world_pose_tf.transform.translation.y;
        world_pose.position.z = world_pose_tf.transform.translation.z;
        world_pose.orientation.x = world_pose_tf.transform.rotation.x;
        world_pose.orientation.y = world_pose_tf.transform.rotation.y;
        world_pose.orientation.z = world_pose_tf.transform.rotation.z;
        world_pose.orientation.w = world_pose_tf.transform.rotation.w;

        return world_pose;
    }

    geometry_msgs::Pose transformToWorldFrame(
        const geometry_msgs::Pose& target,
        std::string agv) {
        static tf2_ros::StaticTransformBroadcaster br;
        geometry_msgs::TransformStamped transformStamped;

        std::string kit_tray;
        if (agv.compare("agv1") == 0)
            kit_tray = "kit_tray_1";
        else if (agv.compare("agv2") == 0)
            kit_tray = "kit_tray_2";
        else if (agv.compare("agv3") == 0)
            kit_tray = "kit_tray_3";
        else if (agv.compare("agv4") == 0)
            kit_tray = "kit_tray_4";

        transformStamped.header.stamp = ros::Time::now();
        transformStamped.header.frame_id = kit_tray;
        transformStamped.child_frame_id = "target_frame";
        transformStamped.transform.translation.x = target.position.x;
        transformStamped.transform.translation.y = target.position.y;
        transformStamped.transform.translation.z = target.position.z;
        transformStamped.transform.rotation.x = target.orientation.x;
        transformStamped.transform.rotation.y = target.orientation.y;
        transformStamped.transform.rotation.z = target.orientation.z;
        transformStamped.transform.rotation.w = target.orientation.w;


        for (int i{ 0 }; i < 15; ++i)
            br.sendTransform(transformStamped);

        tf2_ros::Buffer tfBuffer;
        tf2_ros::TransformListener tfListener(tfBuffer);
        ros::Rate rate(10);
        ros::Duration timeout(1.0);


        geometry_msgs::TransformStamped world_pose_tf;
        geometry_msgs::TransformStamped ee_target_tf;


        for (int i = 0; i < 10; i++) {
            try {
                world_pose_tf = tfBuffer.lookupTransform("world", "target_frame",
                    ros::Time(0), timeout);
            }
            catch (tf2::TransformException& ex) {
                ROS_WARN("%s", ex.what());
                ros::Duration(1.0).sleep();
                continue;
            }
        }

        geometry_msgs::Pose world_pose{};
        world_pose.position.x = world_pose_tf.transform.translation.x;
        world_pose.position.y = world_pose_tf.transform.translation.y;
        world_pose.position.z = world_pose_tf.transform.translation.z;
        world_pose.orientation.x = world_pose_tf.transform.rotation.x;
        world_pose.orientation.y = world_pose_tf.transform.rotation.y;
        world_pose.orientation.z = world_pose_tf.transform.rotation.z;
        world_pose.orientation.w = world_pose_tf.transform.rotation.w;

        return world_pose;
    }

    geometry_msgs::Pose transformToTrayFrame(
        const geometry_msgs::Pose& target,
        std::string agv) {
        static tf2_ros::StaticTransformBroadcaster br;
        geometry_msgs::TransformStamped transformStamped;

        std::string kit_tray;
        if (agv.compare("agv1") == 0)
            kit_tray = "kit_tray_1";
        else if (agv.compare("agv2") == 0)
            kit_tray = "kit_tray_2";
        else if (agv.compare("agv3") == 0)
            kit_tray = "kit_tray_3";
        else if (agv.compare("agv4") == 0)
            kit_tray = "kit_tray_4";

        transformStamped.header.stamp = ros::Time::now();
        transformStamped.header.frame_id = "world";
        transformStamped.child_frame_id = "target_frame";
        transformStamped.transform.translation.x = target.position.x;
        transformStamped.transform.translation.y = target.position.y;
        transformStamped.transform.translation.z = target.position.z;
        transformStamped.transform.rotation.x = target.orientation.x;
        transformStamped.transform.rotation.y = target.orientation.y;
        transformStamped.transform.rotation.z = target.orientation.z;
        transformStamped.transform.rotation.w = target.orientation.w;


        for (int i{ 0 }; i < 15; ++i)
            br.sendTransform(transformStamped);

        tf2_ros::Buffer tfBuffer;
        tf2_ros::TransformListener tfListener(tfBuffer);
        ros::Rate rate(10);
        ros::Duration timeout(1.0);


        geometry_msgs::TransformStamped tray_pose_tf;


        for (int i = 0; i < 10; i++) {
            try {
                tray_pose_tf = tfBuffer.lookupTransform(kit_tray, "target_frame",
                    ros::Time(0), timeout);
            }
            catch (tf2::TransformException& ex) {
                ROS_WARN("%s", ex.what());
                ros::Duration(1.0).sleep();
                continue;
            }
        }

        geometry_msgs::Pose tray_pose{};
        tray_pose.position.x = tray_pose_tf.transform.translation.x;
        tray_pose.position.y = tray_pose_tf.transform.translation.y;
        tray_pose.position.z = tray_pose_tf.transform.translation.z;
        tray_pose.orientation.x = tray_pose_tf.transform.rotation.x;
        tray_pose.orientation.y = tray_pose_tf.transform.rotation.y;
        tray_pose.orientation.z = tray_pose_tf.transform.rotation.z;
        tray_pose.orientation.w = tray_pose_tf.transform.rotation.w;

        return tray_pose; 
    }
  }

  namespace location
  {
      std::string get_pose_location(const geometry_msgs::Pose& pose) {
          double x = pose.position.x; 
          double y = pose.position.y; 


          // Caution: top is smaller than bottom
          double bins_top_x = -3; 
          double bins_middle_x = -2.3; 
          double bins_bottom_x = -1.5; 

          double bins0_right_y = 3.8; 
          double bins0_middle_y = 3; 
          double bins0_left_y = 2.1; 

          if (y > bins0_middle_y and
              y < bins0_right_y and
              x < bins_bottom_x and
              x > bins_middle_x) {

              return "bin1"; 
          }

          if (y > bins0_left_y and
              y < bins0_middle_y and
              x < bins_bottom_x and
              x > bins_middle_x) {

              return "bin2"; 
          }    

          if (y > bins0_left_y  and
              y < bins0_middle_y and
              x < bins_middle_x and
              x > bins_top_x) {

              return "bin3"; 
          }    

          if (y > bins0_middle_y  and
              y < bins0_right_y and
              x < bins_middle_x and
              x > bins_top_x) {

              return "bin4"; 
          }

          double bins1_right_y = -2.1; 
          double bins1_middle_y = -3; 
          double bins1_left_y = -3.8; 

          if (y > bins1_middle_y and
              y < bins1_right_y and
              x < bins_bottom_x and
              x > bins_middle_x) {

              return "bin6"; 
          }

          if (y > bins1_left_y and
              y < bins1_middle_y and
              x < bins_bottom_x and
              x > bins_middle_x) {

              return "bin5"; 
          }    

          if (y > bins1_left_y  and
              y < bins1_middle_y and
              x < bins_middle_x and
              x > bins_top_x) {

              return "bin8"; 
          }    

          if (y > bins1_middle_y  and
              y < bins1_right_y and
              x < bins_middle_x and
              x > bins_top_x) {

              return "bin7"; 
          }

          double ks_top_x = -2.5; 
          double ks_bottom_x = -1.6; 

          double ks1_right_y = 5; 
          double ks1_left_y = 4.3; 

          if (y > ks1_left_y  and
               y < ks1_right_y and
               x < ks_bottom_x and
               x > ks_top_x) {

               return "ks1"; 
           }

          double ks2_right_y = 1.7; 
          double ks2_left_y = 1; 

          if (y > ks2_left_y  and
               y < ks2_right_y and
               x < ks_bottom_x and
               x > ks_top_x) {

               return "ks2"; 
           }

          double ks3_right_y = -1; 
          double ks3_left_y = -1.7;  

          if (y > ks3_left_y  and
               y < ks3_right_y and
               x < ks_bottom_x and
               x > ks_top_x) {

               return "ks3"; 
           }

          double ks4_right_y = -4.3; 
          double ks4_left_y = -5; 

          if (y > ks4_left_y  and
               y < ks4_right_y and
               x < ks_bottom_x and
               x > ks_top_x) {

               return "ks4"; 
           }

          return ""; 
      }

      int get_pose_location_in_bin(const geometry_msgs::Pose& pose, const std::string& bin_id) {
          double bin1_center_x = -1.9; 
          double bin1_center_y = 3.38; 

          double bin2_center_x = -1.9; 
          double bin2_center_y = 2.565; 

          double bin3_center_x = -2.65; 
          double bin3_center_y = 2.565; 

          double bin4_center_x = -2.65; 
          double bin4_center_y = 3.38; 

          double bin5_center_x = -1.9; 
          double bin5_center_y = -3.38; 

          double bin6_center_x = -1.9; 
          double bin6_center_y = -2.565; 

          double bin7_center_x = -2.65; 
          double bin7_center_y = -2.565; 

          double bin8_center_x = -2.65; 
          double bin8_center_y = -3.38;

          double center_x; 
          double center_y; 
          if (bin_id == "bin1") {
              center_x = bin1_center_x; 
              center_y = bin1_center_y; 
          }
          if (bin_id == "bin2") {
              center_x = bin2_center_x; 
              center_y = bin2_center_y; 
          }
          if (bin_id == "bin3") {
              center_x = bin3_center_x; 
              center_y = bin3_center_y; 
          }
          if (bin_id == "bin4") {
              center_x = bin4_center_x; 
              center_y = bin4_center_y; 
          }
          if (bin_id == "bin5") {
              center_x = bin5_center_x; 
              center_y = bin5_center_y; 
          }
          if (bin_id == "bin6") {
              center_x = bin6_center_x; 
              center_y = bin6_center_y; 
          }
          if (bin_id == "bin7") {
              center_x = bin7_center_x; 
              center_y = bin7_center_y; 
          }
          if (bin_id == "bin8") {
              center_x = bin8_center_x; 
              center_y = bin8_center_y; 
          }

          double x = pose.position.x; 
          double y = pose.position.y; 
          // part location
          //     0 1
          //     2 3
          // conveyer belt
          if (x < center_x and y < center_y) {
              return 0;
          }
          if (x < center_x and y > center_y) {
              return 1;
          }
          if (x > center_x and y < center_y) {
              return 2;
          }
          if (x > center_x and y > center_y) {
              return 3; 
          }

      }

      geometry_msgs::Pose get_pose_from_bin_location(const std::string& bin_id, int i) {
          double x, y; 
          double z = 0.75; 
          if (bin_id == "bin1") {
            if (i == 0) {
                x = -1.998; 
                y = 3.28; 
            }
            if (i == 1) {
                x = -1.998; 
                y = 3.48; 
            }
            if (i == 2) {
                x = -1.799; 
                y = 3.28; 
            }
            if (i == 3) {
                x = -1.799; 
                y = 3.48; 
            }
          }

          if (bin_id == "bin2") {
            if (i == 0) {
                x = -1.998; 
                y = 2.465; 
            }
            if (i == 1) {
                x = -1.998; 
                y = 2.665; 
            }
            if (i == 2) {
                x = -1.799; 
                y = 2.465; 
            }
            if (i == 3) {
                x = -1.799; 
                y = 2.665; 
            }
          }

          if (bin_id == "bin4") {
            if (i == 0) {
                x = -2.75; 
                y = 3.28; 
            }
            if (i == 1) {
                x = -2.75; 
                y = 3.48; 
            }
            if (i == 2) {
                x = -2.55; 
                y = 3.28; 
            }
            if (i == 3) {
                x = -2.55; 
                y = 3.48; 
            }
          }

          if (bin_id == "bin3") {
            if (i == 0) {
                x = -2.75; 
                y = 2.465; 
            }
            if (i == 1) {
                x = -2.75; 
                y = 2.665; 
            }
            if (i == 2) {
                x = -2.55; 
                y = 2.465; 
            }
            if (i == 3) {
                x = -2.55; 
                y = 2.665; 
            }
          }

          if (bin_id == "bin6") {
            if (i == 0) {
                x = -1.998; 
                y = -2.665; 
            }
            if (i == 1) {
                x = -1.998; 
                y = -2.465; 
            }
            if (i == 2) {
                x = -1.799; 
                y = -2.665; 
            }
            if (i == 3) {
                x = -1.799; 
                y = -2.465; 
            }
          }

          if (bin_id == "bin5") {
            if (i == 0) {
                x = -1.998; 
                y = -3.48; 
            }
            if (i == 1) {
                x = -1.998; 
                y = -3.28; 
            }
            if (i == 2) {
                x = -1.799; 
                y = -3.48; 
            }
            if (i == 3) {
                x = -1.799; 
                y = -3.28; 
            }
          }

          if (bin_id == "bin7") {
            if (i == 0) {
                x = -2.75; 
                y = -2.665; 
            }
            if (i == 1) {
                x = -2.75; 
                y = -2.465; 
            }
            if (i == 2) {
                x = -2.55; 
                y = -2.665; 
            }
            if (i == 3) {
                x = -2.55; 
                y = -2.465; 
            }
          }

          if (bin_id == "bin8") {
            if (i == 0) {
                x = -2.75; 
                y = -3.48; 
            }
            if (i == 1) {
                x = -2.75; 
                y = -3.28; 
            }
            if (i == 2) {
                x = -2.55; 
                y = -3.48; 
            }
            if (i == 3) {
                x = -2.55; 
                y = -3.28; 
            }
          }

          geometry_msgs::Pose pose; 
          pose.position.x = x; 
          pose.position.y = y; 
          pose.position.z = z; 

          return pose; 
      }
  }
  
}
