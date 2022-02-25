#ifndef ORDER_INFO_H
#define ORDER_INFO_H

#include <vector>
#include <string>

#include <ros/ros.h>

#include <geometry_msgs/Pose.h>


struct Product
{
  Product(const std::string n, const geometry_msgs::Pose p):
    name{n},
    frame_pose{p}{
  }
  std::string name;
  geometry_msgs::Pose frame_pose;
  //geometry_msgs::Pose world_pose;
}; 

struct Shipment
{
  Shipment(const std::string &ship_t, const std::string &a, const std::string &st, const std::vector<Product> &p):
    shipment_type{ship_t},
    agv{a},
    station{st},
    products{std::move(p)}{

  }
  std::string shipment_type;
  std::string agv;
  std::string station;
  std::vector<Product> products; 
}; 

struct Order
{
    std::string order_id;
    std::vector<Shipment> kitting_shipments;
    std::vector<Shipment> assembly_shipments;
}; 

#endif

 
