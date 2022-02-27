#include <ros/ros.h>

#include "agv.h"


int main(int argc, char **argv){

  ros::init(argc, argv, "test_agv"); 

  ros::NodeHandle nh; 

  AGV agv2 = AGV(&nh, "agv2"); 
  //auto shipment_type = "order_0_kitting_shipment_0"; 
  auto station_id = "as2"; 
  //agv2.submit_shipment(shipment_type, station_id); 

  agv2.to_as(station_id); 

  ros::spin(); 
  return 0; 
}
