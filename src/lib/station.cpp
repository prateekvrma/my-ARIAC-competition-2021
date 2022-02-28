#include "station.h"

#include <nist_gear/AssemblyStationSubmitShipment.h>
#include <my_ariac/Busy.h>

using AssemSubmit = nist_gear::AssemblyStationSubmitShipment; 

Station::Station(ros::NodeHandle* nodehandle, const std::string& id):
  m_nh{*nodehandle}, 
  m_id{id},
  m_logical_camera(nodehandle,
                  m_logical_camera_id += id.back()) 
{  
  // Subscribers
  m_competition_state_subscriber = m_nh.subscribe("/ariac/competition_state", 10, &Station::competition_state_callback, this); 
  m_task_subscriber = m_nh.subscribe("/factory_manager/assembly_task", 10, &Station::task_callback, this); 

  // Publishers
  m_busy_publisher = m_nh.advertise<my_ariac::Busy>("/worker/busy", 50); 
}

void Station::competition_state_callback(const std_msgs::String::ConstPtr& msg)
{
  m_competition_state = msg->data; 
}

void Station::task_callback(const nist_gear::AssemblyShipment::ConstPtr& msg)
{
  const std::lock_guard<std::mutex> lock(*m_mutex_ptr); 

  if (msg->station_id == m_id) {
    m_tasks.emplace_back(std::make_unique<nist_gear::AssemblyShipment>(*msg)); 
  }
}

void Station::publish_busy_state()
{
    my_ariac::Busy msg; 
    msg.id = m_id; 
    msg.state = not m_tasks.empty(); 
    m_busy_publisher.publish(msg); 
}

bool Station::get_order()
{
  ros::Rate wait_rate(1); 
  while (m_tasks.empty() && ros::ok()) {
    ROS_INFO_THROTTLE(3, "Waiting for assembly task.");

    this->publish_busy_state(); 
    if (m_competition_state=="done") {
      return false; 
    }

    ros::spinOnce(); 
    wait_rate.sleep(); 
  }
  this->publish_busy_state(); 
}

void Station::plan()
{
  const std::lock_guard<std::mutex> lock(*m_mutex_ptr); 

  for (auto &task_ptr: m_tasks) {
    this->execute_tasks(task_ptr.get()); 
  }

  m_tasks.clear(); 
}

void Station::execute_tasks(const nist_gear::AssemblyShipment* task_ptr)
{
    this->submit_shipment(task_ptr->shipment_type); 
}

void Station::submit_shipment(const std::string& shipment_type)
{  

  auto service_name = "/ariac/" + m_id + "/submit_shipment"; 
  static auto client = m_nh.serviceClient<AssemSubmit>(service_name); 

  if (!client.exists()) {
    ROS_INFO("Waiting for the competition to be ready...");
    client.waitForExistence();
    ROS_INFO("Competition is now ready.");
  }

  AssemSubmit srv; 
  srv.request.shipment_type = shipment_type; 

  if (client.call(srv)) {
    ROS_INFO("Calling service %s", service_name.c_str()); 
    ROS_INFO_STREAM("inspection result: " << srv.response.inspection_result); 
  }
  else{
    ROS_ERROR("Failed to call %s", service_name.c_str()); 
  }

}
