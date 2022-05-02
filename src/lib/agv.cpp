#include "agv.h"

#include <std_srvs/Trigger.h>
#include <nist_gear/AGVToAssemblyStation.h>

using AGVToAssem = nist_gear::AGVToAssemblyStation; 

AGV::AGV(ros::NodeHandle* nodehandle, const std::string &id):
  m_nh{*nodehandle}, 
  m_id{id} 
{  
  m_kitting_station_id += id.back();

  // create subscribers
  m_state_subscriber = m_nh.subscribe("/ariac/" + id + "/state", 10, &AGV::state_callback, this); 
  m_station_subscriber = m_nh.subscribe("/ariac/" + id + "/station", 10, &AGV::station_callback, this); 

  m_submit_shipment_client = 
      m_nh.serviceClient<AGVToAssem>("/ariac/" + m_id + "/submit_shipment"); 
  m_submit_shipment_client.waitForExistence();

}

void AGV::state_callback(const std_msgs::String::ConstPtr& msg)
{
  m_state = msg->data; 
}

void AGV::station_callback(const std_msgs::String::ConstPtr& msg)
{
  m_station = msg->data; 
}

std::string AGV::get_station()
{
    return m_station; 
}

void AGV::submit_shipment(const std::string& shipment_type,  
                          const std::string& station_id)
{
  AGVToAssem srv; 
  srv.request.assembly_station_name = station_id;  
  srv.request.shipment_type = shipment_type; 
  // call the service to allow AGV to submit kitting shipment
  if (m_submit_shipment_client.call(srv)) {
    ROS_INFO("Calling service submit_shipment: %s", shipment_type.c_str()); 
    ROS_INFO("%s", srv.response.message.c_str()); 
  }
  else{
    ROS_ERROR("Failed to submit shipment: %s", shipment_type.c_str()); 
  }

}

void AGV::to_as(const std::string& station_id)
{
  if (!valid_station(m_id, station_id)) {
    ROS_INFO("%s is invalid for %s", station_id.c_str(), m_id.c_str()); 
    return; 
  }

  ros::Rate rate(20); 
  // spinOnce() to receive msg from the subscriber
  while (m_station.empty() && ros::ok()) {
    ros::spinOnce(); 
    rate.sleep(); 
  }

  if (m_station == station_id) {
    ROS_INFO("%s is already in %s.", m_id.c_str(), station_id.c_str()); 
    return; 
  } 
  auto service_name = "/ariac/" + m_id + "/to_" + station_id; 
  static auto client = m_nh.serviceClient<std_srvs::Trigger>(service_name); 
  // check if the client exists
  if (!client.exists()) {
    ROS_INFO("Waiting for the competition to be ready...");
    client.waitForExistence();
    ROS_INFO("Competition is now ready.");
  }

  std_srvs::Trigger srv; 
  // call the service to send AGV to the assembly station  
  if (client.call(srv)) {
    ROS_INFO("Calling service %s", service_name.c_str()); 
    ROS_INFO("%s", srv.response.message.c_str()); 
  }
  else {
    ROS_ERROR("Failed to call %s", service_name.c_str()); 
  }
 
}

bool valid_station(const std::string& agv, const std::string& station)
{
  if (agv=="agv1" || agv=="agv2") {
    return station=="as1" || station=="as2"; 
  }
  if (agv=="agv3" || agv=="agv4") {
    return station=="as3" || station=="as4"; 
  }
}
