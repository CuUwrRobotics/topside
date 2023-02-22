if [ -z "$1" ]
then
      joystick_device='js0'
else
      joystick_device=$1
fi

joystick_file="/dev/input/$joystick_device"

sudo chmod 777 $joystick_file
rosparam set joy_node/dev $joystick_file
rosrun joy joy_node
