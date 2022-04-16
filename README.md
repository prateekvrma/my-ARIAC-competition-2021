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

## RWA3
```
roslaunch ariac_group1 ariac_testing.launch
roslaunch ariac_group1 kitting_arm.launch
roslaunch ariac_group1 rwa3.launch
```


