# Robotic-Arm-Design and Control | ORT-2023


ssh ortrover@192.168.1.3
roscore


export ROS_MASTER_URI=http://192.168.1.3:11311/
export ROS_HOSTNAME=192.168.1.2
rosrun joy joy_node


ssh ortrover@192.168.1.3
rosrun rosserial_python serial_node.py /dev/ttyUSB1


ssh ortrover@192.168.1.3
python3 joy.py
