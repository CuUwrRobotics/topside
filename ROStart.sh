#!/bin/bash
# To modify Baud ./RoStart.sh -b 115300
# Check if a baud rate is provided as a command-line argument
if [ $# -eq 2 ] && [ "$1" = "-b" ]; then
  baud_rate="$2"
else
  baud_rate="115200"  # Default baud rate if not specified
fi

# Change the baud rate by setting it in a ROS parameter
#rosparam set /hardware_driver/baud_rate "$baud_rate"

# Set the permissions for /dev/ttyACM0(theoretically the usb that the arduino is connected to)
sudo chmod 777 /dev/ttyACM0

# Launch necessary ROS nodes in separate terminal tabs
gnome-terminal --tab -- bash -c "roscore"
gnome-terminal --tab -- bash -c "rosrun joy joy_node"
gnome-terminal --tab -- bash -c "rosrun joy_mapper joy_mapper_node"
gnome-terminal --tab -- bash -c "rosrun motor_mapper motor_mapper_node"
gnome-terminal --tab -- bash -c "rosrun hardware_driver hardware_driver_node -p /dev/ttyACM0 -b $baud_rate"
gnome-terminal --tab -- bash -c "rosrun control_loop control_loop_node"
