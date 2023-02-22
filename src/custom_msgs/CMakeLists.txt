cmake_minimum_required(VERSION 3.0.2)
project(custom_msgs)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
find_package(catkin REQUIRED COMPONENTS
  std_msgs
  message_generation
)

add_message_files(
  FILES
  EulerMotion.msg
  ArmJointMotion.msg
  MotorControls.msg
  Leak.msg
)

add_service_files(
  FILES
  Lockout.srv
  ResetMotors.srv
)

generate_messages(
  DEPENDENCIES
  std_msgs
)

###################################
## catkin specific configuration ##
###################################

catkin_package(
  INCLUDE_DIRS
  CATKIN_DEPENDS std_msgs message_generation
  DEPENDS
)

###########
## Build ##
###########

## Specify additional locations of header files
## Your package locations should be listed before other locations
include_directories(
# include
  ${catkin_INCLUDE_DIRS}
)
