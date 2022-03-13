# ARIAC 2021 Competition (In progress)
This project implements the simulation content of ARIAC competition 2021. 

The goal is to control the whole factory system to process the incoming orders, 
and win as many score as it can at the end of simulation.  
This score is depend on the used resources, finish time, success orders,  etc.

More info are provided at the ARIAC github page.   
https://github.com/usnistgov/ARIAC  

Detailed explanation of this project:  
https://longhongc.notion.site/ARIAC-RWA1-dad9e700be6a44698f7eca4c956f02c4

## Install
1. Install the ARIAC environment  
https://github.com/usnistgov/ARIAC/blob/master/wiki/tutorials/installation.md  
Follow all the steps except cloning the modified versin of ARIAC instead.  
```
git clone https://github.com/longhongc/ARIAC
```
The modified ARIAC adds more tasks and fix some issues. 

2. Add this competition package to src
```
cd {your catkin_ws/src}
git clone https://github.com/longhongc/my-ARIAC-competition-2021.git
```
3. Build
```
catkin_make or cakin build
```

## File Structure
```
├── CMakeLists.txt
├── LICENSE
├── README.md
├── config (Config file for ariac environment)
│   ├── trial_config (Order, parts, competition setting)
│   │   ├── rwa2_trial.yaml
│   │   └── task1_sensor_setup.yaml
│   └── user_config (Sensors setting)
│       └── rwa2_sensor.yaml
├── doc
│   └── instructions.txt
├── include (Supporting library headerfile)
│   ├── agv.h
│   ├── factory_manager.h
│   ├── sensors.h
│   └── station.h
├── launch
│   ├── ariac.launch
│   ├── rwa1.launch
│   └── rwa2.launch
├── msg (Custom message)
│   └── Busy.msg
├── package.xml
├── script
│   └── part_spawner.sh (Script for activating events)
└── src
    ├── lib (Supporting library source code)
    │   ├── agv.cpp
    │   ├── factory_manager.cpp
    │   ├── sensors.cpp
    │   └── station.cpp
    ├── nodes (Executable ros node source code)
    │   ├── agv_node.cpp
    │   ├── rwa1.cpp
    │   ├── rwa2.cpp
    │   ├── sensors_node.cpp
    │   └── station_node.cpp
    └── test (Test file source code)
        ├── main.cpp
        ├── test_agv.cpp
        ├── test_sensors.cpp
        └── test_station.cpp
```
## RWA1
### Run
Start ariac simulation environment
```
roslaunch nist_gear sample_environment.launch
```

Start competition nodes
```
roslaunch ariac_group1 rwa1.launch
```

## RWA2
### Run
Start ariac simulation environment
```
roslaunch ariac_group1 ariac.launch
```

Start competition nodes
```
roslaunch ariac_group1 rwa2.launch
```

Trigger events  
```
roscd ariac_group1/script
sh part_spawner.sh
```
Events:  
1. High-priority order  
2. Sensors blackout 10s  
3. Faulty part  
4. Insufficient parts  

After the competition nodes starts, run the above script to trigger events.  
First a high-priority order will be triggered.  
After 5s, sensors blackout will be triggered and last for 10s.  
Once the blackout finishes, faulty part will be detected.  
Insufficient order will be printed out if the parts in an order couldn't be found after 20s it was announced.   
After all the events finish, close the program with Ctrl-C.  
