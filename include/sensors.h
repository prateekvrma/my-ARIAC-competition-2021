/**
 * @file sensors.h
 * @authors Bo-Shiang Wang (bwang24@umd.edu), Chang-Hong Chen (markchen@umd.edu), Prateek Verma (verma@umd.edu), Sparsh Jaiswal (sjaiswal@umd.edu)
 * @brief This header file contains the class definitions and functions for all the sensors used in ARIAC 
 * @version 0.1
 * @date 2022-03-01
 * 
 * @copyright Copyright (c) 2022
 * 
 */

#ifndef SENSORS_H
#define SENSORS_H

#include <string>

#include <ros/ros.h>

#include <nist_gear/LogicalCameraImage.h>
#include <nist_gear/Proximity.h>
#include <sensor_msgs/LaserScan.h>
#include <sensor_msgs/Range.h>
#include <sensor_msgs/PointCloud.h>

/**
 * @brief Class member function for sensors placed in the workcell.
 * 
 */
class Sensors {
  public:
    Sensors(ros::NodeHandle* nodehandle, const std::string& id); 
    virtual ~Sensors() = 0; 

  protected: 
    ros::NodeHandle m_nh; 
    // sensor id
    std::string m_id; 
    ros::Subscriber m_sensor_subscriber; 
}; 

/**
 * @brief Class member function of Logical Camera with built-in object classification and localization system.
 *        - Reports the position and orientation of the camera in the world and collection of the objects detected within its frustum.
 */
class LogicalCamera: private Sensors {
  public:
    LogicalCamera(ros::NodeHandle* nodehandle, const std::string &id); 

  private:

    /**
     * @Brief Subscriber callback function for sensor data. 
     *
     * @Param msg
     */
    void sensor_callback(const nist_gear::LogicalCameraImage::ConstPtr& msg); 
}; 

/**
 * @brief Class member function of the time-of-flight Depth Camera (Swissranger SR4000).
 * 
 */
class DepthCamera: private Sensors {
  public:
    DepthCamera(ros::NodeHandle* nodehandle, const std::string& id); 

  private:

    /**
     * @Brief Subscriber callback function for sensor data. 
     *
     * @Param msg
     */
    void sensor_callback(const sensor_msgs::PointCloud::ConstPtr& msg); 
}; 

/**
 * @brief Class member function of ultrasound Proximity sensor (SU2-A0-0A).
 *        - detects range of ~0.5 meters. 
 * 
 */
class ProximitySensor: private Sensors {
  public:
    ProximitySensor(ros::NodeHandle* nodehandle, const std::string& id); 

  private:

    /**
     * @Brief Subscriber callback function for sensor data. 
     *
     * @Param msg
     */
    void sensor_callback(const sensor_msgs::Range::ConstPtr& msg); 
}; 

/**
 * @brief Class member function of 3D Laser Profiler (Cognex DS1300).
 *        - output is array of ranges and intensities.
 *        - max. range of each beam = ~0.725 meters.  
 */
class LaserProfiler: private Sensors {
  public:
    LaserProfiler(ros::NodeHandle* nodehandle, const std::string& id); 

  private:

    /**
     * @Brief Subscriber callback function for sensor data. 
     *
     * @Param msg
     */
    void sensor_callback(const sensor_msgs::LaserScan::ConstPtr& msg); 
}; 

/**
 * @brief Class member function of photoelectric Break Beam Sensor (Sick W9L-3).
 *        - detects range of 1 meter
 *        - binary output tells whether there is an object crossing the beam.        
 * 
 */
class BreakBeam: private Sensors {
  public:
    BreakBeam(ros::NodeHandle* nodehandle, const std::string& id); 

  private:

    /**
     * @Brief Subscriber callback function for sensor data. 
     *        Callback function recieved continous break_beam data
     *
     * @Param msg
     */
    void sensor_callback(const nist_gear::Proximity::ConstPtr& msg); 
    // callback function when break_beam received a change
    /**
     * @Brief Subscriber callback function for sensor data. 
     *        Callback function triggered when break_beam received a change
     *
     * @Param msg
     */

    void sensor_change_callback(const nist_gear::Proximity::ConstPtr& msg); 
    ros::Subscriber m_sensor_change_subscriber; 
}; 


#endif 
