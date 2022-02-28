#include "agv.h"


#include <std_srvs/Trigger.h>
#include <nist_gear/AGVToAssemblyStation.h>
#include <my_ariac/Busy.h>

using AGVToAssem = nist_gear::AGVToAssemblyStation; 

AGV::AGV(ros::NodeHandle* nodehandle, const std::string &id):
  m_nh{*nodehandle}, 
  m_id{id}, 
  m_quality_control_sensor(nodehandle,
                           m_quality_control_sensor_id += id.back())
{  
  // Subscribers
  m_state_subscriber = m_nh.subscribe("/ariac/" + id + "/state", 10, &AGV::state_callback, this); 
  m_station_subscriber = m_nh.subscribe("/ariac/" + id + "/station", 10, &AGV::station_callback, this); 
  m_competition_state_subscriber = m_nh.subscribe("/ariac/competition_state", 10, &AGV::competition_state_callback, this); 
  m_task_subscriber = m_nh.subscribe("/factory_manager/kitting_task", 10, &AGV::task_callback, this); 

  // Publishers
  m_busy_publisher = m_nh.advertise<my_ariac::Busy>("/worker/busy", 10); 
}

void AGV::state_callback(const std_msgs::String::ConstPtr& msg)
{
  m_state = msg->data; 
}

void AGV::station_callback(const std_msgs::String::ConstPtr& msg)
{
  ROS_INFO("%s: %s", m_id.c_str(), msg->data.c_str());
  m_station = msg->data; 
  ROS_INFO("%s", m_station.c_str()); 
}

void AGV::competition_state_callback(const std_msgs::String::ConstPtr& msg)
{
  m_competition_state = msg->data; 
}

void AGV::task_callback(const nist_gear::KittingShipment::ConstPtr& msg)
{
  const std::lock_guard<std::mutex> lock(*m_mutex_ptr); 

  if (msg->agv_id == m_id) {
    m_tasks.emplace_back(std::make_unique<nist_gear::KittingShipment>(*msg)); 
  }
}

void AGV::publish_busy_state()
{
  my_ariac::Busy msg; 
  msg.id = m_id; 
  msg.state = not m_tasks.empty(); 
  m_busy_publisher.publish(msg); 
}

bool AGV::get_order()
{
  ros::Rate wait_rate(1); 
  while (m_tasks.empty() && ros::ok()) {
    ROS_INFO_THROTTLE(3, "Waiting for kitting task.");

    this->publish_busy_state(); 
    if (m_competition_state=="done") {
      return false; 
    }
    ros::spinOnce(); 
    wait_rate.sleep(); 
  }
  this->publish_busy_state(); 
  return true; 
}

void AGV::plan()
{
  const std::lock_guard<std::mutex> lock(*m_mutex_ptr); 

  for (auto& task_ptr: m_tasks) {
    this->execute_tasks(task_ptr.get()); 
  }

  m_tasks.clear(); 
}

void AGV::execute_tasks(const nist_gear::KittingShipment* task_ptr)
{
    this->submit_shipment(task_ptr->shipment_type,
                          task_ptr->station_id); 
}


void AGV::submit_shipment(const std::string& shipment_type,  
                          const std::string& station_id)
{

  auto service_name = "/ariac/" + m_id + "/submit_shipment"; 
  static auto client = m_nh.serviceClient<AGVToAssem>(service_name); 

  if (!client.exists()) {
    ROS_INFO("Waiting for the competition to be ready...");
    client.waitForExistence();
    ROS_INFO("Competition is now ready.");
  }

  AGVToAssem srv; 
  srv.request.assembly_station_name = station_id;  
  srv.request.shipment_type = shipment_type; 

  if (client.call(srv)) {
    ROS_INFO("Calling service %s", service_name.c_str()); 
    ROS_INFO("%s", srv.response.message.c_str()); 
  }
  else{
    ROS_ERROR("Failed to call %s", service_name.c_str()); 
  }

}

void AGV::to_as(const std::string& station_id)
{
  if (!valid_station(m_id, station_id)) {
    ROS_INFO("%s is invalid for %s", station_id.c_str(), m_id.c_str()); 
    return; 
  }

  ros::Rate rate(20); 
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

  if (!client.exists()) {
    ROS_INFO("Waiting for the competition to be ready...");
    client.waitForExistence();
    ROS_INFO("Competition is now ready.");
  }

  std_srvs::Trigger srv; 

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
