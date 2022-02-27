#include "factory_manager.h"

#include <string>

#include <std_srvs/Trigger.h>

FactoryManager::FactoryManager(ros::NodeHandle* nodehandle):
  m_nh{*nodehandle}{
  m_order_subscriber = m_nh.subscribe("/ariac/orders", 10, &FactoryManager::order_callback, this); 
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
  ROS_INFO_STREAM("vec size: " << m_orders.size()); 

  for(auto &order: m_orders){
    for(auto &shipment: order->kitting_shipments){
      ROS_INFO("%s", shipment.shipment_type.c_str());
      ROS_INFO("%s", shipment.agv_id.c_str());
      ROS_INFO("%s", shipment.station_id.c_str());
    }

    for(auto &shipment: order->assembly_shipments){
      ROS_INFO("%s", shipment.shipment_type.c_str());
      ROS_INFO("%s", shipment.station_id.c_str());
    }
  }

}

// void FactoryManager::assign_kitting_task(Shipment &shipment){
//   std::string topic_name = "/factory_manager/kitting_task" 
//
//   static auto publisher = m_nh.advertise<nist_gear::KittingShipment>(topic_name, 10); 
//
//   nist_gear::KittingShipment msg; 
//   msg.shipment_type = shipment.shipment_type; 
//   msg.agv_id = shipment.agv; 
//   msg.station_id = shipment.station; 
//
//   nist_gear::
//   for(auto &product: shipment.products){
//
//     msg
//   }
//
// }
//
// void FactoryManager::assign_assembly_task(){
//
// }
//
//
