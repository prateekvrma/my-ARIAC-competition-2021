#ifndef ORDER_INFO_H
#define ORDER_INFO_H

#include <vector>
#include <string>

#include <ros/ros.h>

#include <geometry_msgs/Pose.h>


struct Product
{
    std::string name;
    geometry_msgs::Pose frame_pose;
    geometry_msgs::Pose world_pose;
    std::string agv;
    std::string kit_tray;
}; 

struct Shipment
{
    std::string shipment_type;
    std::string agv;
}; 

struct Order
{
    std::string order_id;
    std::vector<Shipment> shipments;
}; 

#endif

 
