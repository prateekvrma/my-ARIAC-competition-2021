#include "sensors.h"

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
  //std::cout << m_id << std::endl; 
  ROS_INFO_THROTTLE(1, "Callback triggered for Topic %s", m_id.c_str());
  //ROS_INFO_THROTTLE(3, "%s: %d objects.", m_id.c_str(), (int)msg->models.size());
  //ROS_INFO("%s: %d objects.", m_id.c_str(), (int)msg->models.size()); 
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
