#include "agv.h"

#include <std_srvs/Trigger.h>
#include <nist_gear/AGVToAssemblyStation.h>

using AGVToAssem = nist_gear::AGVToAssemblyStation; 

AGV::AGV(ros::NodeHandle* nodehandle, const std::string &id):
  m_nh{*nodehandle}, 
  m_id{id}{
}

void AGV::submit_shipment(const std::string &shipment_type,  
                          const std::string &station_id){

  auto service_name = "/ariac/" + m_id + "/submit_shipment"; 
  static auto client = m_nh.serviceClient<AGVToAssem>(service_name); 

  if (!client.exists())
  {
    ROS_INFO("Waiting for the competition to be ready...");
    client.waitForExistence();
    ROS_INFO("Competition is now ready.");
  }

  AGVToAssem srv; 
  srv.request.assembly_station_name = station_id;  
  srv.request.shipment_type = shipment_type; 

  if (client.call(srv)){
    ROS_INFO("%s", srv.response.message.c_str()); 
  }
  else{
    ROS_ERROR("Failed to call %s", service_name.c_str()); 
  }

}

void AGV::to_as(const std::string &station_id){

  auto service_name = "/ariac/" + m_id + "/to_" + station_id; 
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
