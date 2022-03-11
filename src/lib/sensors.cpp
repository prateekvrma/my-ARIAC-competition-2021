#include "sensors.h"

#include <tf2_ros/transform_listener.h>
#include <geometry_msgs/TransformStamped.h>
#include <geometry_msgs/Pose.h>
#include <geometry_msgs/Quaternion.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h> //--needed for tf2::Matrix3x3

#include <nist_gear/Model.h>

Sensors::Sensors(ros::NodeHandle* nodehandle, const std::string &id): 
  m_nh{*nodehandle}, 
  m_id{id} {}

Sensors::~Sensors() {}

LogicalCamera::LogicalCamera(ros::NodeHandle* nodehandle, const std::string& id): 
  Sensors(nodehandle, id)
{
    m_sensor_subscriber = m_nh.subscribe("/ariac/" + id, 10, &LogicalCamera::sensor_callback, this); 
}

void LogicalCamera::sensor_callback(const nist_gear::LogicalCameraImage::ConstPtr& msg)
{
  tf2_ros::Buffer tfBuffer;
  tf2_ros::TransformListener tfListener(tfBuffer);

  ros::Duration timeout(5.0);
  geometry_msgs::TransformStamped transformStamped;
  try {
    ROS_INFO("%s", m_id.c_str()); 
    transformStamped = tfBuffer.lookupTransform("world", m_id + "_frame",
                                                 ros::Time(0), timeout);
  }
  catch (tf2::TransformException& ex) {
    ROS_WARN("%s", ex.what());
    ros::Duration(1.0).sleep();
    return; 
  }

  tf2::Quaternion camera_q_tf(
    transformStamped.transform.rotation.x,
    transformStamped.transform.rotation.y,
    transformStamped.transform.rotation.z,
    transformStamped.transform.rotation.w);

  for (auto &model: msg->models) {
    geometry_msgs::Pose model_pose = model.pose; 
    geometry_msgs::Pose transformed_model_pose; 

    tf2::doTransform(model_pose, transformed_model_pose, transformStamped); 

    // calculate orientation in world frame
    tf2::Quaternion model_q_tf;
    tf2::convert(transformed_model_pose.orientation, model_q_tf); 
    model_q_tf.normalize(); 

    //convert Quaternion to Euler angles
    tf2::Matrix3x3 m(model_q_tf);
    double roll, pitch, yaw;
    m.getRPY(roll, pitch, yaw);

    ROS_INFO("%s in /world frame: [%f,%f,%f] [%f,%f,%f]",
      model.type.c_str(), 
      transformed_model_pose.position.x,
      transformed_model_pose.position.y,
      transformed_model_pose.position.z,
      roll,
      pitch,
      yaw);
    }

}

DepthCamera::DepthCamera(ros::NodeHandle* nodehandle, const std::string& id):
  Sensors(nodehandle, id)
{
    m_sensor_subscriber = m_nh.subscribe("/ariac/" + id, 10, &DepthCamera::sensor_callback, this); 
}

void DepthCamera::sensor_callback(const sensor_msgs::PointCloud::ConstPtr& msg)
{
   //ROS_INFO_STREAM_THROTTLE(10, m_id);
   //ROS_INFO_THROTTLE(1, "Callback triggered for Topic %s", m_id.c_str());
   return; 
}

ProximitySensor::ProximitySensor(ros::NodeHandle* nodehandle, const std::string& id): 
  Sensors(nodehandle, id)
{
    m_sensor_subscriber = m_nh.subscribe("/ariac/" + id, 10, &ProximitySensor::sensor_callback, this); 
}

void ProximitySensor::sensor_callback(const sensor_msgs::Range::ConstPtr& msg)
{
  ROS_INFO_THROTTLE(1, "Callback triggered for Topic %s", m_id.c_str());
  if ((msg->max_range - msg->range) > 0.01) {
    // If there is an object in proximity.
    ROS_INFO_THROTTLE(1, "%s: sensed object", m_id.c_str());
  }
}

LaserProfiler::LaserProfiler(ros::NodeHandle* nodehandle, const std::string& id): 
  Sensors(nodehandle, id)
{
    m_sensor_subscriber = m_nh.subscribe("/ariac/" + id, 10, &LaserProfiler::sensor_callback, this); 
}

void LaserProfiler::sensor_callback(const sensor_msgs::LaserScan::ConstPtr& msg)
{
  // size_t number_of_valid_ranges = std::count_if(
  //   msg->ranges.begin(), msg->ranges.end(), [](const float f)
  //   {
  //     return std::isfinite(f);
  //   });
  // if (number_of_valid_ranges > 0)
  // {
  //   ROS_INFO_THROTTLE(1, "%s: sensed object", m_id.c_str());
  // }
  ROS_INFO_THROTTLE(1, "Callback triggered for Topic %s", m_id.c_str());
}

BreakBeam::BreakBeam(ros::NodeHandle* nodehandle, const std::string& id): 
  Sensors(nodehandle, id)
{
    m_sensor_subscriber = m_nh.subscribe("/ariac/" + id, 10, &BreakBeam::sensor_callback, this); 
    m_sensor_change_subscriber = m_nh.subscribe("/ariac/" + id + "_change", 10, &BreakBeam::sensor_change_callback, this); 
}

void BreakBeam::sensor_callback(const nist_gear::Proximity::ConstPtr& msg)
{
    ROS_INFO_THROTTLE(1, "Callback triggered for Topic %s", m_id.c_str());
    if (msg->object_detected) {  // If there is an object in proximity.
      ROS_INFO("%s triggered.", m_id.c_str());
    }
}

void BreakBeam::sensor_change_callback(const nist_gear::Proximity::ConstPtr& msg)
{
    ROS_INFO_THROTTLE(1, "Callback triggered for Topic %s", m_id.c_str());
    if (msg->object_detected) {  // If there is an object in proximity.
      ROS_INFO("%s triggered.", (m_id + "_change").c_str());
    }
}
