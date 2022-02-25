#include "factory_manager.h"

#include <string>

#include <std_srvs/Trigger.h>

FactoryManager::FactoryManager(ros::NodeHandle* nodehandle):
  m_nh{*nodehandle}{
  m_order_subscriber = m_nh.subscribe("/ariac/orders", 10, &FactoryManager::order_callback, this); 
}

void FactoryManager::order_callback(const nist_gear::Order::ConstPtr & msg){

  Order order{};
  order.order_id = msg->order_id; 

  for(auto &shipment: msg->kitting_shipments){
    std::vector<Product> products; 
    for(auto &product: shipment.products){
      products.emplace_back(product.type, product.pose); 
    }

    ROS_INFO("%s", shipment.shipment_type.c_str());
    ROS_INFO("%s", shipment.agv_id.c_str());
    ROS_INFO("%s", shipment.station_id.c_str());

    order.kitting_shipments.emplace_back(shipment.shipment_type, 
                                         shipment.agv_id,
                                         shipment.station_id, 
                                         products); 
  }

  for(auto &shipment: msg->assembly_shipments){
    std::vector<Product> products; 
    for(auto &product: shipment.products){
      products.emplace_back(product.type, product.pose); 
    }

    ROS_INFO("%s", shipment.shipment_type.c_str());
    ROS_INFO("%s", shipment.station_id.c_str());

    std::string agv = "None"; 
    order.assembly_shipments.emplace_back(shipment.shipment_type, 
                                          agv, 
                                          shipment.station_id, 
                                          products); 
  }

  m_orders.push_back(std::move(order)); 
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
