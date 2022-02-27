#include "factory_manager.h"

#include <string>

#include <std_srvs/Trigger.h>
#include <nist_gear/KittingShipment.h>
#include <nist_gear/AssemblyShipment.h>


FactoryManager::FactoryManager(ros::NodeHandle* nodehandle):
  m_nh{*nodehandle}{
  m_order_subscriber = m_nh.subscribe("/ariac/orders", 10, &FactoryManager::order_callback, this); 

  m_kitting_publisher = m_nh.advertise<nist_gear::KittingShipment>("/factory_manager/kitting_task", 10); 
  m_assembly_publisher = m_nh.advertise<nist_gear::AssemblyShipment>("/factory_manager/assembly_task", 10); 


}

void FactoryManager::order_callback(const nist_gear::Order::ConstPtr & msg){

  m_orders.emplace_back(std::make_unique<nist_gear::Order>(*msg)); 

}

void FactoryManager::start_competition(){
  std::string service_name = "/ariac/start_competition"; 

  static auto client = m_nh.serviceClient<std_srvs::Trigger>(service_name); 

  if (!client.exists())
  {
    ROS_INFO("Waiting for the competition to be ready...");
    client.waitForExistence();
    ROS_INFO("Competition is now ready.");
  }

  std_srvs::Trigger srv; 

  if (client.call(srv)){
    ROS_INFO("%s", srv.response.message.c_str()); 
  }
  else{
    ROS_ERROR("Failed to call %s", service_name.c_str()); 
  }
 
}

void FactoryManager::end_competition(){
  std::string service_name = "/ariac/end_competition"; 

  static auto client = m_nh.serviceClient<std_srvs::Trigger>(service_name); 

  if (!client.exists())
  {
    ROS_INFO("Waiting for the competition to be ready...");
    client.waitForExistence();
    ROS_INFO("Competition is now ready.");
  }

  std_srvs::Trigger srv; 

  if (client.call(srv)){
    ROS_INFO("%s", srv.response.message.c_str()); 
  }
  else{
    ROS_ERROR("Failed to call %s", service_name.c_str()); 
  }

} 

void FactoryManager::plan(){
  ROS_INFO_STREAM("Orders: " << m_orders.size()); 

  for(auto &order: m_orders){
    for(auto &shipment: order->kitting_shipments){
      this->assign_kitting_task(shipment); 
    }

    for(auto &shipment: order->assembly_shipments){
      this->assign_assembly_task(shipment); 
    }
  }

}

void FactoryManager::assign_kitting_task(nist_gear::KittingShipment &shipment){
  nist_gear::KittingShipment msg; 
  msg = shipment; 

  ROS_INFO("kitting task"); 
  m_kitting_publisher.publish(msg); 
}

void FactoryManager::assign_assembly_task(nist_gear::AssemblyShipment &shipment){
  nist_gear::AssemblyShipment msg; 
  msg = shipment; 

  ROS_INFO("assembly task"); 
  m_assembly_publisher.publish(msg); 
}


