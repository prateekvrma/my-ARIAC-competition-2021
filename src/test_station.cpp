#include <string>

#include <ros/ros.h>

#include "station.h"


int main(int argc, char **argv){

  ros::init(argc, argv, "test_station"); 

  ros::NodeHandle nh; 

  Station as2 = Station(&nh, "as2"); 
  std::string shipment_type = "order_0_assembly_shipment_0"; 
  as2.submit_shipment(shipment_type); 

  return 0; 
}
