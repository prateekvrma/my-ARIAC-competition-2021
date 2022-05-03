cmake_minimum_required(VERSION 3.0.2)
project(ariac_group1)

## Compile as C++11, supported in ROS Kinetic and newer
#add_compile_options(-std=c++11)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  control_msgs
  nist_gear
  roscpp
  rospy
  rosgraph_msgs
  sensor_msgs
  std_srvs
  trajectory_msgs
  geometry_msgs
  tf2
  tf2_eigen
  tf2_ros
  tf2_geometry_msgs
  message_generation
  moveit_msgs
  moveit_core
  moveit_ros_perception
  moveit_ros_planning
  moveit_ros_planning_interface
  moveit_visual_tools
)

find_package(Eigen3 REQUIRED)
find_package(Boost REQUIRED system filesystem date_time thread)
## System dependencies are found with CMake's conventions
# find_package(Boost REQUIRED COMPONENTS system)


## Uncomment this if the package has a setup.py. This macro ensures
## modules and global scripts declared therein get installed
## See http://ros.org/doc/api/catkin/html/user_guide/setup_dot_py.html
# catkin_python_setup()

################################################
## Declare ROS messages, services and actions ##
################################################

## To declare and build messages, services or actions from within this
## package, follow these steps:
## * Let MSG_DEP_SET be the set of packages whose message types you use in
##   your messages/services/actions (e.g. std_msgs, actionlib_msgs, ...).
## * In the file package.xml:
##   * add a build_depend tag for "message_generation"
##   * add a build_depend and a exec_depend tag for each package in MSG_DEP_SET
##   * If MSG_DEP_SET isn't empty the following dependency has been pulled in
##     but can be declared for certainty nonetheless:
##     * add a exec_depend tag for "message_runtime"
## * In this file (CMakeLists.txt):
##   * add "message_generation" and every package in MSG_DEP_SET to
##     find_package(catkin REQUIRED COMPONENTS ...)
##   * add "message_runtime" and every package in MSG_DEP_SET to
##     catkin_package(CATKIN_DEPENDS ...)
##   * uncomment the add_*_files sections below as needed
##     and list every .msg/.srv/.action file to be processed
##   * uncomment the generate_messages entry below
##   * add every package in MSG_DEP_SET to generate_messages(DEPENDENCIES ...)

## Generate messages in the 'msg' folder
add_message_files(
  FILES
  Busy.msg
  PartInfo.msg
  Database.msg
  KittingInfo.msg
  PartTask.msg
)

## Generate services in the 'srv' folder
add_service_files(
  FILES
  GetParts.srv
  GetShipmentPriority.srv
  IsFaulty.srv
  IsShipmentReady.srv
  PartsInCamera.srv
  IsPartPicked.srv
  GetPartPosition.srv
  CheckQualitySensor.srv
  GetCompetitionTime.srv
  GetVacancyPose.srv
  GetBeltPart.srv
  GetBeltProximitySensor.srv
  PartsUnderCamera.srv
)


## Generate actions in the 'action' folder
# add_action_files(
#   FILES
#   Action1.action
#   Action2.action
# )

## Generate added messages and services with any dependencies listed here
generate_messages(
  DEPENDENCIES
  std_msgs
  nist_gear
  #sensor_msgs#   trajectory_msgs
)

################################################
## Declare ROS dynamic reconfigure parameters ##
################################################

## To declare and build dynamic reconfigure parameters within this
## package, follow these steps:
## * In the file package.xml:
##   * add a build_depend and a exec_depend tag for "dynamic_reconfigure"
## * In this file (CMakeLists.txt):
##   * add "dynamic_reconfigure" to
##     find_package(catkin REQUIRED COMPONENTS ...)
##   * uncomment the "generate_dynamic_reconfigure_options" section below
##     and list every .cfg file to be processed

## Generate dynamic reconfigure parameters in the 'cfg' folder
# generate_dynamic_reconfigure_options(
#   cfg/DynReconf1.cfg
#   cfg/DynReconf2.cfg
# )

###################################
## catkin specific configuration ##
###################################
## The catkin_package macro generates cmake config files for your package
## Declare things to be passed to dependent projects
## INCLUDE_DIRS: uncomment this if your package contains header files
## LIBRARIES: libraries you create in this project that dependent projects also need
## CATKIN_DEPENDS: catkin_packages dependent projects also need
## DEPENDS: system dependencies of this project that dependent projects also need
catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES group1_rwa1
#  CATKIN_DEPENDS nist_gear roscpp rospy sensor_msgs std_srvs trajectory_msgs
#  DEPENDS system_lib
)

###########
## Build ##
###########

include_directories(include ${catkin_INCLUDE_DIRS})

add_library(utility_lib src/lib/utility.cpp)
add_library(sensors_lib src/lib/sensors.cpp src/lib/utility.cpp)
add_library(station_lib src/lib/station.cpp src/lib/sensors.cpp src/lib/utility.cpp)
add_library(agv_lib src/lib/agv.cpp src/lib/sensors.cpp src/lib/utility.cpp)
add_library(kitting_arm_lib src/lib/kitting_arm.cpp src/lib/utility.cpp src/lib/shipments.cpp src/lib/agv.cpp)
add_library(gantry_arm_lib src/lib/gantry_arm.cpp src/lib/utility.cpp src/lib/assembly_shipments.cpp src/lib/agv.cpp)
add_library(sensor_manager_lib src/lib/sensor_manager.cpp src/lib/sensors.cpp src/lib/utility.cpp) 
add_library(orders_lib src/lib/orders.cpp)
add_library(shipments_lib src/lib/shipments.cpp)
add_library(factory_manager_lib src/lib/factory_manager.cpp src/lib/orders.cpp) 


add_dependencies(utility_lib ${catkin_EXPORTED_TARGETS})
add_dependencies(sensors_lib ${catkin_EXPORTED_TARGETS})
add_dependencies(station_lib ${catkin_EXPORTED_TARGETS})
add_dependencies(agv_lib ${catkin_EXPORTED_TARGETS})
add_dependencies(kitting_arm_lib ${catkin_EXPORTED_TARGETS})
add_dependencies(gantry_arm_lib ${catkin_EXPORTED_TARGETS})
add_dependencies(sensor_manager_lib ${catkin_EXPORTED_TARGETS})
add_dependencies(orders_lib ${catkin_EXPORTED_TARGETS})
add_dependencies(shipments_lib ${catkin_EXPORTED_TARGETS})
add_dependencies(factory_manager_lib ${catkin_EXPORTED_TARGETS})

add_executable(${PROJECT_NAME}_rwa2_node src/nodes/rwa2.cpp)
add_dependencies(${PROJECT_NAME}_rwa2_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(${PROJECT_NAME}_rwa2_node
  factory_manager_lib
  ${catkin_LIBRARIES}
)

add_executable(${PROJECT_NAME}_sensor_manager_node src/nodes/sensor_manager.cpp)
add_dependencies(${PROJECT_NAME}_sensor_manager_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(${PROJECT_NAME}_sensor_manager_node
  sensor_manager_lib
  ${catkin_LIBRARIES}
)

add_executable(${PROJECT_NAME}_kitting_arm_node src/nodes/kitting_arm.cpp)
add_dependencies(${PROJECT_NAME}_kitting_arm_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(${PROJECT_NAME}_kitting_arm_node
  kitting_arm_lib
  ${catkin_LIBRARIES}
)

add_executable(${PROJECT_NAME}_kitting_arm_test_node src/test/test_kitting_arm.cpp)
add_dependencies(${PROJECT_NAME}_kitting_arm_test_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(${PROJECT_NAME}_kitting_arm_test_node
  kitting_arm_lib
  ${catkin_LIBRARIES}
)

add_executable(${PROJECT_NAME}_test_utils_node src/test/test_utils.cpp)
add_dependencies(${PROJECT_NAME}_test_utils_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(${PROJECT_NAME}_test_utils_node
  utility_lib
  ${catkin_LIBRARIES}
)

add_executable(${PROJECT_NAME}_gantry_arm_node src/nodes/gantry_arm.cpp)
add_dependencies(${PROJECT_NAME}_gantry_arm_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(${PROJECT_NAME}_gantry_arm_node
  gantry_arm_lib
  ${catkin_LIBRARIES}
)

add_executable(${PROJECT_NAME}_gantry_arm_test_node src/test/test_gantry_arm.cpp)
add_dependencies(${PROJECT_NAME}_gantry_arm_test_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(${PROJECT_NAME}_gantry_arm_test_node
  gantry_arm_lib
  ${catkin_LIBRARIES}
)
 
