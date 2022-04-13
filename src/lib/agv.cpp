#include "agv.h"


#include <std_srvs/Trigger.h>
#include <nist_gear/AGVToAssemblyStation.h>
#include <ariac_group1/Busy.h>

#include <ariac_group1/GetShipmentPriority.h>
#include <ariac_group1/PartTask.h>

using AGVToAssem = nist_gear::AGVToAssemblyStation; 

ShipmentInfo::ShipmentInfo(const std::string& id,
                           const nist_gear::KittingShipment::ConstPtr& shipment_ptr):
  shipment_id{id},
  shipment{std::make_unique<nist_gear::KittingShipment>(*shipment_ptr)}
{
}

AGV::AGV(ros::NodeHandle* nodehandle, const std::string &id):
  m_nh{*nodehandle}, 
  m_id{id}, 
  m_quality_control_sensor(nodehandle,
                           m_quality_control_sensor_id += id.back())
{  
  m_kitting_station_id += id.back();

  // create subscribers
  m_state_subscriber = m_nh.subscribe("/ariac/" + id + "/state", 10, &AGV::state_callback, this); 
  m_station_subscriber = m_nh.subscribe("/ariac/" + id + "/station", 10, &AGV::station_callback, this); 
  m_competition_state_subscriber = m_nh.subscribe("/ariac/competition_state", 10, &AGV::competition_state_callback, this); 
  m_task_subscriber = m_nh.subscribe("/factory_manager/kitting_task", 10, &AGV::task_callback, this); 
  // create publisher 
  m_busy_publisher = m_nh.advertise<ariac_group1::Busy>("/worker/busy", 10); 
  m_part_task_publisher = m_nh.advertise<ariac_group1::PartTask>("/part_task", 10); 
}

void AGV::state_callback(const std_msgs::String::ConstPtr& msg)
{
  m_state = msg->data; 
}

void AGV::station_callback(const std_msgs::String::ConstPtr& msg)
{
  m_station = msg->data; 
}

void AGV::competition_state_callback(const std_msgs::String::ConstPtr& msg)
{
  m_competition_state = msg->data; 
}

void AGV::task_callback(const nist_gear::KittingShipment::ConstPtr& msg)
{
  const std::lock_guard<std::mutex> lock(*m_mutex_ptr); 
  // add tasks to task vector
  if (msg->agv_id == m_id) {
    m_new_shipments_id.push_back(msg->shipment_type); 
    m_shipments_id.push_back(msg->shipment_type); 
    m_shipments_record[msg->shipment_type] = std::make_unique<ShipmentInfo>(msg->shipment_type, msg); 
  }
}

void AGV::publish_busy_state()
{
  // setup parameters to state that AGV is busy
  ariac_group1::Busy msg; 
  msg.id = m_id; 
  msg.state = false; 
  for (auto& id: m_shipments_id) {
    if (m_shipments_record[id]->state != ShipmentState::Finish) {
      msg.state = true; 
      break; 
    }
  }
  m_busy_publisher.publish(msg); 
}

bool AGV::get_order()
{
  ros::Rate wait_rate(1); 
  // check if there are tasks and make sure ROS is running 
  while (m_new_shipments_id.empty() && ros::ok()) {
    ROS_INFO_THROTTLE(3, "Waiting for kitting task.");

    this->publish_busy_state(); 
    // AGV will not receive task when the competition ended
    if (m_competition_state=="done") {
      return false; 
    }
    ros::spinOnce(); 
    wait_rate.sleep(); 
  }

  this->publish_busy_state(); 
  ROS_INFO("Received kitting task"); 
  return true; 
}

void AGV::plan()
{
  const std::lock_guard<std::mutex> lock(*m_mutex_ptr); 

  std::string service_name = "/orders/get_shipment_priority"; 
  static auto client = m_nh.serviceClient<ariac_group1::GetShipmentPriority>(service_name); 

  if (!client.exists()) {
    ROS_INFO("Waiting for the get_shipment_priority_service..");
    client.waitForExistence();
    ROS_INFO("Shipment priority is now ready.");
  }
   


  for (auto& id: m_new_shipments_id) {
    for (auto& product: m_shipments_record[id]->shipment->products) {
      ariac_group1::PartTask part_task; 
      part_task.shipment_type = id; 
      part_task.part = product;  
      part_task.total_parts = m_shipments_record[id]->shipment->products.size(); 
      part_task.agv_id = m_id; 
      part_task.station_id = m_shipments_record[id]->shipment->shipment_type; 

      ariac_group1::GetShipmentPriority get_shipment_priority_srv; 
      get_shipment_priority_srv.request.shipment_type = id; 

      if (client.call(get_shipment_priority_srv)) {
        part_task.priority = get_shipment_priority_srv.response.priority; 
        m_part_task_publisher.publish(part_task);  
      }
      else {
        ROS_ERROR("Failed to call %s", service_name.c_str()); 
      }
    }
    // this->execute_tasks(task_ptr.get()); 
  }

  m_new_shipments_id.clear(); 
}

// void AGV::check_shipments()
// {
//   for (auto& id: m_shipments_id) {
//     if (m_shipments_record[id]->state != ShipmentState::Finish) {
//       if () {
//         ROS_INFO("Shipment %s is ready", id.c_str()); 
//         m_shipments_record[id]->state = ShipmentState::Ready; 
//         this->execute_tasks(m_shipments_record[id]->shipment.get()); 
//       }
//     }
//   }
//
// }

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
  // check if the client exists
  if (!client.exists()) {
    ROS_INFO("Waiting for the competition to be ready...");
    client.waitForExistence();
    ROS_INFO("Competition is now ready.");
  }

  AGVToAssem srv; 
  srv.request.assembly_station_name = station_id;  
  srv.request.shipment_type = shipment_type; 
  // call the service to allow AGV to submit kitting shipment
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
