# ARIAC 2021 Competition (In progress)
This project implements the simulation content of ARIAC competition 2021. 

The goal is to control the whole factory system to process the incoming orders, 
and win as many score as it can at the end of simulation.  
This score is depend on the used resources, finish time, success orders,  etc.

More info are provided at the ARIAC github page.   
https://github.com/usnistgov/ARIAC  

# Install
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

# File Structure
```
├── CMakeLists.txt  
├── README.md  
├── doc  
│   └── instructions.txt  
├── include (Supporint library headerfile)
│   ├── agv.h  
│   ├── factory_manager.h  
│   ├── group1_rwa1  
│   ├── sensors.h  
│   └── station.h  
├── launch 
│   └── group1_rwa1.launch 
├── msg (Custom message)  
│   └── Busy.msg  
├── package.xml  
└── src  
    ├── lib (Supporting library source code) 
    │   ├── agv.cpp  
    │   ├── factory_manager.cpp  
    │   ├── sensors.cpp
    │   └── station.cpp
    ├── nodes (Executable ros node source code) 
    │   ├── agv_node.cpp  
    │   ├── group1_rwa1.cpp  
    │   ├── sensors_node.cpp  
    │   └── station_node.cpp  
    └── test (Test file source code)  
        ├── main.cpp  
        ├── test_agv.cpp  
        ├── test_sensors.cpp  
        └── test_station.cpp
```
# Run
Start ariac simulation environment
```
roslaunch nist_gear sample_environment.launch
```
Start competition nodes
```
roslaunch my_ariac group1_rwa1.launch
```


