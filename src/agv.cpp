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

  AGVToAssem srv; 
  srv.request.assembly_station_name = station_id;  
  srv.request.shipment_type = shipment_type; 

  if (client.call(srv)){
    //auto srv_sucess_msg = "AGV[" + m_name + "] finish ["
    //                      + shipment_type + "] to [" + assembly_station_name + "]";  
    ROS_INFO("%s", srv.response.message.c_str()); 
  }
  else{
    ROS_ERROR("Failed to call %s", service_name.c_str()); 
  }

}

void AGV::to_as(const std::string &station_id){
  auto service_name = "/ariac/" + m_id + "/to_" + station_id; 
  static auto client = m_nh.serviceClient<std_srvs::Trigger>(service_name); 

  std_srvs::Trigger srv; 

  if (client.call(srv)){
    //auto srv_sucess_msg = "AGV[" + m_name + "] finish ["
    //                      + shipment_type + "] to [" + assembly_station_name + "]";  
    ROS_INFO("%s", srv.response.message.c_str()); 
  }
  else{
    ROS_ERROR("Failed to call %s", service_name.c_str()); 
  }
 
}
