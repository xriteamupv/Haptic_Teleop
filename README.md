# Haptic Teleoperation Experience

Video AI-based Teleoperation of UR5e Robot Arm and OnRobot RG2 Gripper using ROS packages and complemented with Haptic Feedback from bHaptics TactGloves

## Project Description

The present project aims to develop an immersive teleoperation system for the remote control of a robotic arm. The system allows a user to manipulate remote objects imitating the movements of their own arm through movement capture sensors. The experience is enriched by haptic feedback, providing the user with a real contact sensation with the manipulated object.

<img width="612" alt="Diagram_System_EN" src="https://github.com/xriteamupv/Haptic_Teleop/assets/38531693/aabecd7d-0569-4d68-b8c7-eccac3815dea">

### The system is composed of three main nodes:

#### (1) Robotic arm and gripper controller node:

This node houses the UR5e robotic arm controller, the OnRobot RG2 gripper and the Polyscope communications interface. It is responsible for transmitting information from states (arm position, trajectories result, grip amplitude and strength), as well as receiving the movement instructions from the movement capture node and sending them to the robotic arm for their execution.

#### (2) Movement capture and tracking node: 

This node captures the movements of the human arm through a camera and the ``mediapipe`` library, or a tracking device. The movement data is processed and sent to the master controller node, indicating the robotic arm how to move using ROS and the ``rospy`` library to imitate user movements.

#### (3) Haptic feedback management node:

This node receives the force feedback information of the RG2 gripper from the robotic arm together with the result of the grip recognition. The information is processed and sent to the bHaptics tactile gloves using bHaptics Player and the ``tact-python`` library, providing the user with a sensory vibro-tactile feedback when the robotic arm contacts the manipulated object.

## Hardware Requirements:

- Universal Robots UR5e robot arm or similar
- OnRobot RG2 gripper or similar
- 1 computer for Node (2) for ROS and Mediapipe
- 1 Video Camera compatible with Node (2) computer
- 1 VIVE Trackers 3.0 or similar (optional)
- 1 computer for Node (3) for bHaptics Player
- 1 pair of bHaptics TactGloves

## User Manual

### (A) Turn on and connect all devices from Nodes (1), (2), (3)

### (B) Put your TactGloves on and open the bHaptics Player - Node (3)

### (C) Open a terminal and run the tactile_ctrl.py file - Node (3)

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
