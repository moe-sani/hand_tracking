# Hand Tracking
This is a ROS package that recieves data from Hand Tracking Board and extracts joint angles, and finally maps them to the joints in shadow_robot's hand.

## Run
First we should run ROS core:
'roscore'
## Running the raw sensor data publisher node
in another terminal, run:
'rosrun hand_tracking serial_json_publisher_node.py'
note that data comming from the board should be in JSON format
## Running the joint angles node
'rosrun hand_tracking hand_joint_mapping.py'
## Running the data saver node
'rosrun hand_tracking save_to_file_all.py'

## Install
installation guide will go here
