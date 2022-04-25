#include "factory_manager.h"

#include <string>

#include <std_srvs/Trigger.h>
#include <nist_gear/KittingShipment.h>
#include <nist_gear/AssemblyShipment.h>

FactoryManager::FactoryManager(ros::NodeHandle* nodehandle):
  m_nh{*nodehandle},
  m_orders{nodehandle}
{
  // Subscribers
  m_busy_subscriber = m_nh.subscribe("/worker/busy", 50, &FactoryManager::busy_callback, this); 

  // Publishers
  m_kitting_publisher = m_nh.advertise<nist_gear::KittingShipment>("/factory_manager/kitting_task", 10); 
  m_assembly_publisher = m_nh.advertise<nist_gear::AssemblyShipment>("/factory_manager/assembly_task", 10); 

  // Services
  m_get_competition_time_service = m_nh.advertiseService("/factory_manager/get_competition_time", &FactoryManager::get_competition_time, this); 

  // All workers are free at start
  for (auto& worker: m_workers) {
    m_busy_state[worker] = false; 
  }

}

void FactoryManager::busy_callback(const ariac_group1::Busy& msg){
  m_busy_state[msg.id] = msg.state; 
}

void FactoryManager::run_competition()
{
  this->start_competition(); 
  m_start_time = ros::Time::now(); 

  while (ros::ok()) {

    if (m_orders.get_order()) {

      this->plan(); 

    }
  }
  this->end_competition(); 
}

void FactoryManager::start_competition()
{
  std::string service_name = "/ariac/start_competition"; 

  static auto client = m_nh.serviceClient<std_srvs::Trigger>(service_name); 

  if (!client.exists()) {
    ROS_INFO("Waiting for the competition to be ready...");
    client.waitForExistence();
    ROS_INFO("Competition is now ready.");
  }

  std_srvs::Trigger srv; 

  if (client.call(srv)) {
    ROS_INFO("%s", srv.response.message.c_str()); 
  }
  else {
    ROS_ERROR("Failed to call %s", service_name.c_str()); 
  }
 
}

void FactoryManager::end_competition()
{
  std::string service_name = "/ariac/end_competition"; 

  static auto client = m_nh.serviceClient<std_srvs::Trigger>(service_name); 

  if (!client.exists()) {
    ROS_INFO("Waiting for the competition to be ready...");
    client.waitForExistence();
    ROS_INFO("Competition is now ready.");
  }

  std_srvs::Trigger srv; 

  if (client.call(srv)) {
    ROS_INFO("%s", srv.response.message.c_str()); 
  }
  else {
    ROS_ERROR("Failed to call %s", service_name.c_str()); 
  }

} 

void FactoryManager::plan() 
{
  // Lock to prevent adding new orders when assigning tasks
  const std::lock_guard<std::mutex> lock(*m_mutex_ptr); 

  for (auto& order_id: m_orders.get_new_orders_id()) {
    // Check if parts valid in order
    // auto valid = this->check_order(order_id); 
    auto& order = m_orders.orders_record[order_id]->order; 

    for (auto &shipment: order->kitting_shipments) {
      this->assign_kitting_task(shipment); 
    }

    for (auto &shipment: order->assembly_shipments) {
      this->assign_assembly_task(shipment); 
    }
  }

  m_orders.clear_new_orders_id(); 
}

void FactoryManager::assign_kitting_task(nist_gear::KittingShipment& shipment)
{
  nist_gear::KittingShipment msg; 
  msg = shipment; 

  m_kitting_publisher.publish(msg); 
}

void FactoryManager::assign_assembly_task(nist_gear::AssemblyShipment& shipment){
  nist_gear::AssemblyShipment msg; 
  msg = shipment; 

  m_assembly_publisher.publish(msg); 
}

bool FactoryManager::work_done()
{
  for (const auto& worker_busy_state: m_busy_state) {
    if (worker_busy_state.second) {
      ROS_INFO("Waiting for worker: %s", worker_busy_state.first.c_str());
      return false; 
    }
  }
  return true; 
}

bool FactoryManager::get_competition_time(ariac_group1::GetCompetitionTime::Request &req,
                                         ariac_group1::GetCompetitionTime::Response &res)
{
  auto competition_time = ros::Time::now() - m_start_time; 
  res.competition_time = competition_time.toSec(); 

  return true; 
}
