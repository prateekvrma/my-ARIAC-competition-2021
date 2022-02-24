#include "agv.h"

#include <nist_gear/AGVToAssemblyStation.h>

using AGVToAssem = nist_gear::AGVToAssemblyStation; 

AGV::AGV(ros::NodeHandle* nodehandle, const std::string &id):
  m_nh{*nodehandle}, 
  m_id{id}{
}

void AGV::submit_shipment(const std::string &shipment_type,  
                          const std::string &station_id){

  auto agv_service_name = "/ariac/agv" + m_id + "/submit_shipment"; 
  static auto client = m_nh.serviceClient<AGVToAssem>(agv_service_name); 

  AGVToAssem srv; 
  srv.request.assembly_station_name = station_id;  
  srv.request.shipment_type = shipment_type; 

  if (client.call(srv)){
    //auto srv_sucess_msg = "AGV[" + m_name + "] finish ["
    //                      + shipment_type + "] to [" + assembly_station_name + "]";  
    ROS_INFO("%s", srv.response.message.c_str()); 
  }
  else{
    ROS_ERROR("Failed to call %s", agv_service_name.c_str()); 
  }

}
