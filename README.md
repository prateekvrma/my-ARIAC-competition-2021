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
Might need to build 2~3 times because of custom messages and services.    

## Run 
### RWA4
```
roslaunch ariac_group1 ariac_testing.launch trial_config:=rwa4/complete
roslaunch ariac_group1 arms.launch  
rslaunch ariac_group1 rwa4.launch  
```

The trial config argument can be any file name under the folder config/trial_config without the yaml ending.   

###Other trial configs: 
flip: testing for fliping part 
- half_flip: flip part only 90 degrees    
- sensor_blackout: dealing with sensor blackout during flip part   
- on_agv: dealing with flip part on the agv    

shipment: shipment error handling  
- swap: swap parts between agv and bins  
- wrong_pose: fix wrong pose part on agv     
- wrong_type fix wrong type part on agv      

insufficient: insufficient shipments handling
- high_priority: insufficient high_priority order   
- multi: multiple insufficient orders  
- steal: high_priority order steal order from others causing insufficient order  





