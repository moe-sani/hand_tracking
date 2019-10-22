# Hand Tracking
author: Mohammad Fattahi Sani
Smartsurg project
This is a ROS package that recieves data from Hand Tracking Board and extracts joint angles, and finally maps them to the joints in davinci tool.

## How to Run
there are two ways to run this package, either using launch file or running nodes separately:


### launch file

    roslaunch handtracking HFT.launch

### manual running
First we should run ROS core:

    roscore

then load parameters by navigate to the source folder and:

    rosparam load params.yaml

Now you can run every desired node in a separate terminal

# Functionality of each node
    
the following picture shows all nodes and messages in this package
![Alt text](rosgraph.png?raw=true "Title")

## 1. sensor data publisher node
in another terminal, run:

    rosrun hand_tracking serial_json_publisher_node.py

### Inputs:
this node reads the serial port:

    ser = serial.Serial('/dev/ttyACM0', 115200, timeout=10)
make sure that the port number is correct.
note that data coming from the board should be in JSON format

following is a sample data which is coming from the board.

    {"S0":[-9946,-246,73,995], "S1":[-7496,-3971,-4799,2236], "S2":[532,45,9882,1432], "S3":[-8701,-2219,-4312,874] }
    
    { "C0":[3,3,0,2], "C1":[3,3,3,2], "C2":[3,3,3,2], "C3":[2,3,0,3]} 

first row is quaternions for each sensor and second row is calibration values.

### outputs:
this node publishes the sensor data in /imu_pub_array as PoseArray:

    header: 
      seq: 972
      stamp: 
        secs: 1571486952
        nsecs: 838644981
      frame_id: "/world"
    poses: 
      - 
        position: 
          x: 3.08130956209
          y: -0.909135457577
          z: 0.528552204954
        orientation: 
          x: -0.8703
          y: -0.2218
          z: -0.4307
          w: 0.0885
      - 
        position: 
          x: 0.0240004589185
          y: -0.103482438349
          z: 2.85213680695
        orientation: 
          x: 0.0529
          y: 0.0044
          z: 0.9882
          w: 0.1434
      - 
        position: 
          x: 3.0324268296
          y: -1.11309232465
          z: 1.081537372
        orientation: 
          x: -0.7416
          y: -0.4117
          z: -0.476
          w: 0.2318
      - 
        position: 
          x: -2.93889944297
          y: 0.0168501173629
          z: -0.196843504657
        orientation: 
          x: -0.9899
          y: 0.0986
          z: -0.0016
          w: 0.1015

please note that values under 'position' are orientation in euler angles, and values under 'orientation' are orientations in quaternions.

## 2. joint angles publisher

    rosrun hand_tracking HFT_joint_angle_publisher.py
this node actually calculates the relative quaternion for each joint    
### inputs:
subscribes to /imu_pub_array
### outputs:
publishes relative quaternions to \joints_array

    header: 
      seq: 310804
      stamp: 
        secs: 1571487483
        nsecs: 260601997
      frame_id: "/world"
    poses: 
      - 
        position: 
          x: 3.07201809983
          y: -0.908246332873
          z: 0.489473620726
        orientation: 
          x: 0.875036726062
          y: 0.202808512052
          z: 0.432918169957
          w: -0.0759031857236
      - 
        position: 
          x: -2.37994579936
          y: 0.726156535383
          z: -2.07401404351
        orientation: 
          x: -0.327913518767
          y: 0.814272509369
          z: -0.131362027052
          w: 0.460626771415
      - 
        position: 
          x: 3.02492388927
          y: -1.1289861289
          z: 1.05718821589
        orientation: 
          x: 0.744039881607
          y: 0.398421355957
          z: 0.486026051695
          w: -0.226812157457
      - 
        position: 
          x: -0.896474423976
          y: -0.406938311735
          z: 1.44548295836
        orientation: 
          x: -0.197873188902
          y: -0.417320265314
          z: 0.518125782185
          w: 0.719885873668

the fact that which joint is calculated relative to which joint is given in the params.yaml file.

## 3. joint mapping node

    rosrun hand_tracking HFT_joint_mapping.py
        
this node is responsible for mapping the calculated joint angles to the davinci instrument
### inputs: 
\joints_array 

### outputs:

this node publishes /davinci_joint_states as JointState message type:

    header: 
      seq: 166024
      stamp: 
        secs: 1571487974
        nsecs: 422708988
      frame_id: "world"
    name: [roll, pitch, yaw, jaw]
    position: [1.5707963267948966, -1.1936126033250167, -1.5707963267948966, 1.5707963267948966]
    velocity: []
    effort: []



## 4. davinci driver
this node subscribes to the joint states and publishes the message required by Epos driver node

## outputs:
publishes to /exoeskeleton topic

    layout: 
      dim: []
      data_offset: 0
    data: [50000.0, 60798.359051997904, 46179.95490956216, -100000.0, 1.0]


### 5. epos dirver

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
*** note that there should be space after each :

Write this to test:

    rostopic pub /davinci_joint_states sensor_msgs/JointState "header:
      seq: 0
      stamp: {secs: 0, nsecs: 0}
      frame_id: ''
    name: ['roll','pitch','yaw','jaw']
    position: [0,0,0,0]
    velocity: [0]
    effort: [0]" -r 100

### calibrating the joint limits
joint limit parameters are in yaml file. they should be manually calibrated.
you should run the EPOS driver node and the aforementioned topic manually.
then you should change the values for each joint in order to find the maximum and minimum step counts for the joints


# pedals
pedal can defined in param file.
if you dont have pedal, you can test using the following code.

to engage:

    rostopic pub /pedals std_msgs/Int32 3
to release:

    rostopic pub /pedals std_msgs/Int32 0

# DEBUGGING OPTIONS

## 1. RVIZ visualization

open rviz and load the HFT.rviz file.
you will see the markers and joint limits during the calibration

![Alt text](rviz.png?raw=true "Title")

## 2. rqt plot options

you can see all the joint angles and raw sensor values in rqt plot environment.

![Alt text](rqt.png?raw=true "Title")

## 3. runing the da vinci simulator
you should first install the davinci research kit in another workspace, then
in new terminal, run:

first source the bash file of the desired workspace for the terminal:

    source ~/catkin_ws/devel_release/setup.bash

then run the following command:

    roslaunch dvrk_model arm_rviz_moe.launch arm:=PSM2

dont forget that you should run the main launch file with sim:=true parameter.

