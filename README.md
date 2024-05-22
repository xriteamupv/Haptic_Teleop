# Haptic Teleoperation Experience

Video AI-based Teleoperation of UR5e Robot Arm and OnRobot RG2 Gripper using ROS packages and complemented with Haptic Feedback from bHaptics TactGloves

## Project Description

The present project aims to develop an immersive teleoperation system for the remote control of a robotic arm. The system allows a user to manipulate remote objects imitating the movements of their own arm through movement capture sensors. The experience is enriched by haptic feedback, providing the user with a real contact sensation with the manipulated object.

<img width="1257" alt="DiagramaBloques_TFM" src="https://github.com/xriteamupv/Haptic_Teleop/assets/38531693/9e5c0d8f-71b3-4309-9508-f7687ed8bd68">

### The system is composed of three main nodes:

#### (1) Robotic arm and gripper controller node:

This node houses the UR5e robotic arm controller, the OnRobot RG2 gripper and the Polyscope communications interface. It is responsible for transmitting information from states (arm position, trajectories result, grip amplitude and strength), as well as receiving the movement instructions from the movement capture node and sending them to the robotic arm for their execution.

#### (2) Movement capture node: 

This node captures the movements of the human arm through a camera and the ``mediapipe`` library, or a tracking device. The movement data is processed and sent to the master controller node, indicating the robotic arm how to move using ROS and the ``rospy`` library to imitate user movements.

#### (3) Haptic feedback node:

This node receives the strength information of the RG2 clamp of the robotic arm together with the result of the grip recognition. The information is processed and sent to the bHaptics tactile gloves using bHaptics Player and the ``tact-python`` library, providing the user with a sensory vibro-tactile feedback when the robotic arm contacts the manipulated object.

## System Requirements:

### Hardware:

- Universal Robots UR5e robot arm
- OnRobot RG2 gripper
- 1 computer for Node (2) with Video Camera
- 1 computer for Node (3) with Bluetooth

###  Software for Node (1):
- Polyscope 5.16.0
- OnRobot RG2 URCap
- External Control URCap

### Software for Node (2):
- ROS Noetic v.1.16.0
- Base OS: Ubuntu 20.04
- Python 3.8.3 (Python 3)

### Software for Node (3):
- bHaptics Player v.2.3.3
- Base OS: Microsoft Windows 11
- Python 3.8.3 (Python 3)

## Installation:

### 1. Create workspace in Node (2):
``
mkdir -p ~/catkin_ws/src
``

### 2. Add Universal Robot package:
````
cd ~/catkin_ws/src
git clone https://github.com/fmauch/universal_robot.git
cd universal_robot
git reset --hard 1ffdd69181389b14b7d6342f0c5bad3b45c5e32f
````
### 3. Add OnRobot RG2 package:
TODO

### 4. Install Python libraries:
Use ``pip`` to install the default Python 3 versions of the libraries.

#### Node (2) - see requirements.txt for versions 
````
pip install numpy
pip install rospy
pip install actionlib
pip install datetime
pip install pymodbus
pip install opencv-python
pip install mediapipe
pip install tensorflow
````

#### Node (3)
````
pip install tact-python
pip install datetime
````

### 5. Add haptic_teleop packages:
````
cd ~/catkin_ws/src
git clone https://github.com/xriteamupv/Haptic_Teleop.git
````

Node (2) -> Use ``haptic_u5e`` package. Remove the rest.

Node (3) -> Use ``haptic_glove`` package. Remove the rest.

### 6. Compile packages - Node (2)
````
cd ~/catkin_ws
catkin_make
````

## User Manual

### (A) Turn on and connect all devices from Nodes (1), (2), (3)

### (B) Put your TactGloves on and open the bHaptics Player - Node (3)

### (C) Open a terminal and run the tact_glove.py file - Node (3)

This operation will open the communication between Node (3) and Node (2), and connect to bHaptics SDK.

````
cd ~/haptic_glove
py tact_glove.py <param1> <value1> <param2> <value2> <param3> <value3> ...
````

Note: Change ``<paramX>`` and ``<valueX>`` as needed if you want to modify default application.

### (D) Source and launch ROS package for UR5e communication - Node (2)

If successfull, this operation will set the communication between Node (2) and Node (1).

````
cd ~/catkin_ws
source devel/setup.bash
roslaunch ur_robot_driver ur5e_bringup.launch robot_ip:=<insert_robot_ip> kinematics_config:=<insert_calibration_file.yaml>
````
Note: Change ``<insert_robot_ip>`` and ``<insert_calibration_file.yaml>`` as needed.

### (E) Start program grip_control.urp in Polyscope - Node (1)

This program contains the Variables Setup, the Robot Program with instructions to connect to host, and the RG Gripper initializer. 

It should be previously loaded via USB to the UR5e Teach Pendant containing Polyscope.

### (F) Open other terminal and run the robot_control.py file - Node (2)

This operation will open the communication between the Tracking Operation and the Robots Movement Controllers.

````
cd ~/catkin_ws
source devel/setup.bash
roslaunch haptic_u5e robot_control.py <param1> <value1> <param2> <value2> <param3> <value3> ...
````
Note: Change ``<paramX>`` and ``<valueX>`` as needed if you want to modify default application.

### (G) Open another terminal and run the arm_tracking.py file - Node (2)

This operation will activate the whole system. By default, a camera window should be displayed, which will start tracking the user's hand.

````
cd ~/catkin_ws
source devel/setup.bash
roslaunch haptic_u5e arm_tracking.py <param1> <value1> <param2> <value2> <param3> <value3> ...
````
Note: Change ``<paramX>`` and ``<valueX>`` as needed if you want to modify default application.

### (H) Enjoy the experience!
