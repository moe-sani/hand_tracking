# Hand Tracking
This is a ROS package that recieves data from Hand Tracking Board and extracts joint angles, and finally maps them to the joints in davinci tool.

## Run
First we should run ROS core:

    roscore


## load parameters
navigate to the source folder and:

    rosparam load params.yaml
    
    
## Running the raw sensor data publisher node
in another terminal, run:

    rosrun hand_tracking serial_json_publisher_node.py
    
note that data comming from the board should be in JSON format
## Running the joint angles publisher

    rosrun hand_tracking HFT_joint_angle_publisher.py
## Running the joint mapping

    rosrun hand_tracking HFT_joint_mapping.py
        
## Running the data saver node

    rosrun hand_tracking save_to_file_all.py

## runing the davinci simulation noe

in new terminal, run:

    source catkin_ws/devel_release/setup.bash

    roslaunch dvrk_model arm_rviz_moe.launch arm:=PSM2


## Run the NDI tracker:

    rosrun ndi_tracker_ros ndi_tracker -s /dev/ttyUSB0 -j ~/catkin_ws/src/cisst-saw/sawNDITracker/share/polaris-moe.json



## Running the simulation:
first source the bash file for the terminal:

    source ~/catkin_ws/devel_release/setup.bash

then run the following command:

    roslaunch dvrk_model arm_rviz_moe.launch arm:=PSM2



## davinci driver

### epos dirver

    lsusb

    sudo chmod a+w /dev/bus/usb/001/007


    rosrun exoskeleton mydavinci_node

This subscribes to message /exoskeleton and needs a float64multiarry

rostopic pub /exoskeleton std_msgs/Float64MultiArray "layout: 
  dim: 
    - 
      label: ''
      size: 0
      stride: 0
  data_offset: 0
data: [0.0, 0.0, 0.0, 0.0, 1.0]"


Write this to test:

rostopic pub /davinci_joint_states sensor_msgs/JointState "header:
  seq: 0
  stamp: {secs: 0, nsecs: 0}
  frame_id: ''
name: ['roll','pitch','yaw','jaw']
position: [0,0,0,0]
velocity: [0]
effort: [0]" -r 100



## Install
installation guide will go here
