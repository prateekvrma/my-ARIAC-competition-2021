#ifndef SENSORS_H
#define SENSORS_H

#include <string>

#include <ros/ros.h>

#include <nist_gear/LogicalCameraImage.h>
#include <nist_gear/Proximity.h>
#include <sensor_msgs/LaserScan.h>
#include <sensor_msgs/Range.h>
#include <sensor_msgs/PointCloud.h>

class Sensors{
  public:
    Sensors(ros::NodeHandle* nodehandle, const std::string &id); 
    virtual ~Sensors() = 0; 

  protected: 
    ros::NodeHandle m_nh; 
    std::string m_id; 
    ros::Subscriber m_sensor_subscriber; 
}; 

class LogicalCamera: private Sensors{
  public:
    LogicalCamera(ros::NodeHandle* nodehandle, const std::string &id); 
    void sensor_callback(const nist_gear::LogicalCameraImage::ConstPtr & msg); 
}; 

class DepthCamera: private Sensors{
  public:
    DepthCamera(ros::NodeHandle* nodehandle, const std::string &id); 
    void sensor_callback(const sensor_msgs::PointCloud::ConstPtr & msg); 
}; 

class ProximitySensor: private Sensors{
  public:
    ProximitySensor(ros::NodeHandle* nodehandle, const std::string &id); 
    void sensor_callback(const sensor_msgs::Range::ConstPtr & msg); 
}; 

class LaserProfiler: private Sensors{
  public:
    LaserProfiler(ros::NodeHandle* nodehandle, const std::string &id); 
    void sensor_callback(const sensor_msgs::LaserScan::ConstPtr & msg); 
}; 

class BreakBeam: private Sensors{
  public:
    BreakBeam(ros::NodeHandle* nodehandle, const std::string &id); 
    void sensor_callback(const nist_gear::Proximity::ConstPtr & msg); 
}; 


#endif 
