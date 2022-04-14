#ifndef SENSORS_H
#define SENSORS_H

#include <vector>
#include <memory>
#include <string>
#include <mutex>
#include <map>

#include <ros/ros.h>
#include <geometry_msgs/TransformStamped.h>

#include <nist_gear/LogicalCameraImage.h>
#include <nist_gear/Proximity.h>
#include <nist_gear/Model.h>
#include <sensor_msgs/LaserScan.h>
#include <sensor_msgs/Range.h>
#include <sensor_msgs/PointCloud.h>

#include <ariac_group1/PartInfo.h>

#include "utility.h"

using PartsDB = std::map<std::string, std::vector<std::unique_ptr<ariac_group1::PartInfo>>>;  

/**
 * @brief Class member function for sensors placed in the workcell.
 * 
 */
class Sensors {
  public:
    Sensors(ros::NodeHandle* nodehandle, const std::string& id); 
    virtual ~Sensors()=0; 
    void test_blackout(); 
    bool is_blackout(); 

  protected: 
    ros::NodeHandle m_nh; 
    // sensor id
    std::string m_id; 
    ros::Subscriber m_sensor_subscriber; 
    bool m_blackout = true; 
}; 

/**
 * @brief Class member function of Logical Camera with built-in object classification and localization system.
 *        - Reports the position and orientation of the camera in the world and collection of the objects detected within its frustum.
 */
class LogicalCamera: public Sensors {
  public:
    LogicalCamera(ros::NodeHandle* nodehandle, const std::string &id); 

    /**
     * @Brief Find all the products with the same type as the target under this camera  
     *
     * @Param product_type: the type of a part
     *
     * @Returns the number of same product type found  
     */
    int find_parts(const std::string& product_type); 
    void update_parts(PartsDB& parts_database); 

    std::vector<std::unique_ptr<nist_gear::Model>> parts_world_frame; 

  private:

    /**
     * @Brief Subscriber callback function for sensor data. 
     *
     * @Param msg
     */
    void sensor_callback(const nist_gear::LogicalCameraImage::ConstPtr& msg); 

    /**
     * @Brief Transform the stored parts pose to pose with respect to the world frame  
     */
    void camera_to_world(); 

    ros::Publisher m_parts_publisher; 

    /**
     * @Brief Camera frame with respect to the world frame
     */
    geometry_msgs::TransformStamped m_camera_frame;

    /**
     * @Brief Parts pose with respect to the camera frame 
     */
    std::vector<std::unique_ptr<nist_gear::Model>> m_parts_camera_frame; 

    /**
     * @Brief Parts pose with respect to the world frame  
     */
    double m_platform_height = -1; 

    std::unique_ptr<std::mutex> m_mutex_ptr = std::make_unique<std::mutex>(); 
}; 

/**
 * @brief Class member function of the time-of-flight Depth Camera (Swissranger SR4000).
 * 
 */
class DepthCamera: public Sensors {
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
class ProximitySensor: public Sensors {
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
class LaserProfiler: public Sensors {
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
class BreakBeam: public Sensors {
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
