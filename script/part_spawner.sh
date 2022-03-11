#!/bin/bash

# spawn a red pump in the tray on agv1
rosrun gazebo_ros spawn_model -sdf -x 0.1 -y 0.1 -z 0.05 -R 0 -P 0 -Y 0 -file `rospack find nist_gear`/models/assembly_pump_red_ariac/model.sdf -reference_frame agv1::kit_tray_1::kit_tray_1::tray -model assembly_pump_red_11

sleep 5

# spawn a blue battery in the tray on agv3
rosrun gazebo_ros spawn_model -sdf -x -0.1 -y -0.1 -z 0.05 -R 0 -P 0 -Y 0 -file `rospack find nist_gear`/models/assembly_battery_blue_ariac/model.sdf -reference_frame agv3::kit_tray_3::kit_tray_3::tray -model assembly_battery_blue_5
