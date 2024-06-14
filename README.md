# Haptic Teleoperation Experience

Video AI-based Teleoperation of UR5e Robot Arm and OnRobot RG2 Gripper using ROS packages and complemented with Haptic Feedback from bHaptics TactGloves

## Project Description

The present project aims to develop an immersive teleoperation system for the remote control of a robotic arm. The system allows a user to manipulate remote objects imitating the movements of their own arm through movement capture sensors. The experience is enriched by haptic feedback, providing the user with a real contact sensation with the manipulated object.

<img width="1254" alt="General_Diagram_TFM_Final_v2" src="https://github.com/xriteamupv/Haptic_Teleop/assets/38531693/4cdc82d7-0b7d-4abd-a32f-d2a7e680fe35">

### The system is composed of TWO MAIN NODES:

#### (1) Local Control and Orchestration Node (see [LCON](https://github.com/xriteamupv/Haptic_Teleop/tree/main/LCON_Local_Node)):

This node houses the UR5e robotic arm controller, the OnRobot RG2 gripper and the Polyscope communications interface, thus existing in the same physical location as the robot. 

It is responsible for transmitting information from states (arm position, trajectories result, grip width and strength) using ROS packages, ``rospy`` library and XML-RPC as well as receiving the movement instructions from the remote camera tracking and sending them to the robotic arm and gripper for their execution. This is possible due to an additional URScript program loaded in the UR Teach Pendant, which uses 2 Threads in order to enable simultaneous External Robot Control and Gripper Manipulation.

Finally, it also sends gripper information, such as conditional object detection, width variation and initial force, to the remote haptic program for further processing.

#### (2) Remote Teleoperation and Haptic Node (see [RTHN](https://github.com/xriteamupv/Haptic_Teleop/tree/main/RTHN_Remote_Node)): 

This node contains the Tracking Device, like a 3D camera or specialized tracker, and the bHaptics TactGloves DK1, both of which can be operated far from the Local Node and physical robots.

It captures the movements of the human arm using the ``mediapipe`` library and recognizes several possibilities of grip gestures through a customized CNN that also differentiates between grips of 2, 3, 4 and 5 fingers using ``tensorflow`` for a more immersive experience. The movement and gesture data is processed and sent to the Local Node, while afterwards receiving gripper characteristics which are mapped to vibration intensity values and sent as complex haptic sensations for the tactile gloves using bHaptics Player and the ``tact-python`` library.

## Hardware Requirements:

- Universal Robots UR5e robot arm or similar (see [specifications](https://www.universal-robots.com/es/productos/robot-ur5))
- OnRobot RG2 gripper or similar (see [specifications](https://onrobot.com/en/products/rg2-gripper))
- 1 computer for Local Node LCON with ROS and Ubuntu OS
- 1 computer for Remote Node RTHN with Windows OS and bHaptics Player
- 1 Video Camera capable of 3D rendering, compatible with RTHN computer
- 1 VIVE Trackers 3.0 or similar (optional, see [specifications](https://www.vive.com/eu/accessory/tracker3/))
- 1 pair of bHaptics TactGloves DK1 or similar (see [specifications](https://www.bhaptics.com/shop/tactglove))

## User Manual

### (A) Turn on and connect all devices from LCON and RTHN.

### (B) RTHN -> Put your TactGloves on and link them to bHaptics Player

### (C) RTHN -> Open a terminal and run the haptic_control.py file

This operation will open the receiving end of the communication between nodes, and connect to bHaptics SDK.

````
cd <folder_location>/haptic_ur5e
python haptic_control.py <param1> <value1> <param2> <value2> <param3> <value3> ...
````

#### Success Indicator: 

Output ``LINK ESTABLISHED. Waiting for commands from Local Node ...``

Note: Change ``<paramX>`` and ``<valueX>`` as needed if you want to modify default application. See customization parameters [here](https://github.com/xriteamupv/Haptic_Teleop/tree/main/RTHN_Remote_Node/haptic_ur5e).

### (D) LCON -> Launch ROS package for URXe and XML-RPC for RG2 communication

If successfull, this operation will set the communication between nodes, and connect with Robots through Polyscope.

````
cd <ros_workspace>
. init_controllers.sh --model <ur_model> --robot_ip <robot_ip> --calibration <calibration_file.yaml>
````
#### Success Indicator:

3 tabs will open in the same window, successfully running UR Robot Arm bringup, XML-RPC server and Gripper Controller.

Note: Change ``<ur_model>``, ``<robot_ip>`` and ``<calibration_file.yaml>`` as needed if you want to modify default application.

### (E) Teach Pendant -> Load and start program rg2_remote_control.urp in Polyscope

This program contains the Variables Setup, the Robot Program with instructions to connect to host, and the RG Gripper initializer. It should be previously loaded via USB to the UR5e Teach Pendant containing Polyscope.

#### Success Indicator: 
Output (...) ``Robot requested program`` (...) ``Sent program to robot`` (...) ``Robot ready to receive commands`` on URXe bringup console.

### (F) LCON -> Run the robot_control.py and gripper_control.py programs

This operation will established the communication between the nodes and connect to the local Robots Movement Controllers.

````
cd <ros_workspace>
. init_local_node.sh <param1> <value1> <param2> <value2> <param3> <value3> ...
````
#### Success Indicator: 

2 tabs will open in the same window, one for each program. 

Output ``LINK ESTABLISHED. Waiting for commands from Tracking Program ...``

Note: Change ``<paramX>`` and ``<valueX>`` as needed if you want to modify default application. See customization parameters [here](https://github.com/xriteamupv/Haptic_Teleop/tree/main/LCON_Local_Node/haptic_ur5e/src).

### (G) RTHN -> Open another terminal and run the tracking_control.py program

This operation will activate the whole system. By default, a camera window should be displayed, which will start tracking the user's hand.

````
cd <folder_location>/haptic_ur5e
python tracking_control.py <param1> <value1> <param2> <value2> <param3> <value3> ...
````

#### Success Indicator: 

A camera window appears and starts the hand movements tracking.

Output ``System started, currently tracking your hand ...``

Note: Change ``<paramX>`` and ``<valueX>`` as needed if you want to modify default application. See customization parameters [here](https://github.com/xriteamupv/Haptic_Teleop/tree/main/RTHN_Remote_Node/haptic_ur5e).

### (H) Enjoy the experience!

<img src="images/TBBT_RobotArm2.gif" width="500"/>
