# Haptic Teleoperation Experience

Video AI-based Teleoperation of UR5e Robot Arm and OnRobot RG2 Gripper using ROS packages and complemented with Haptic Feedback from bHaptics TactGloves

## Project Description

The present project aims to develop an immersive teleoperation system for the remote control of a robotic arm. The system allows a user to manipulate remote objects imitating the movements of their own arm through movement capture sensors. The experience is enriched by haptic feedback, providing the user with a real contact sensation with the manipulated object.

<img width="1394" alt="General_Diagram_TFM_Final" src="https://github.com/xriteamupv/Haptic_Teleop/assets/38531693/d96a29e7-910c-428e-a2cd-e5657121c2e8">

### The system is composed of THREE MAIN NODES:

#### (1) Robotic arm and gripper controller node (see [gripper_ur5e](https://github.com/xriteamupv/Haptic_Teleop/tree/main/01_gripper_ur5e)):

This node houses the UR5e robotic arm controller, the OnRobot RG2 gripper and the Polyscope communications interface. It is responsible for transmitting information from states (arm position, trajectories result, grip amplitude and strength), as well as receiving the movement instructions from the movement capture node and sending them to the robotic arm for their execution.

#### (2) Movement capture and tracking node (see [haptic_ur5e](https://github.com/xriteamupv/Haptic_Teleop/tree/main/02_haptic_ur5e)): 

This node captures the movements of the human arm through a camera and the ``mediapipe`` library, or a tracking device. The movement data is processed and sent to the master controller node, indicating the robotic arm how to move using ROS and the ``rospy`` library to imitate user movements.

#### (3) Haptic feedback management node (see [haptic_gloves](https://github.com/xriteamupv/Haptic_Teleop/tree/main/03_haptic_gloves)):

This node receives the force feedback information of the RG2 gripper from the robotic arm together with the result of the grip recognition. The information is processed and sent to the bHaptics tactile gloves using bHaptics Player and the ``tact-python`` library, providing the user with a sensory vibro-tactile feedback when the robotic arm contacts the manipulated object.

## Hardware Requirements:

- Universal Robots UR5e robot arm or similar (see [specifications](https://www.universal-robots.com/es/productos/robot-ur5)
- OnRobot RG2 gripper or similar (see [specifications](https://onrobot.com/en/products/rg2-gripper))
- 1 computer for Node (2) for ROS and Mediapipe
- 1 Video Camera compatible with Node (2) computer
- 1 VIVE Trackers 3.0 or similar (optional, see [specifications](https://www.vive.com/eu/accessory/tracker3/))
- 1 computer for Node (3) for bHaptics Player
- 1 pair of bHaptics TactGloves DK2 or similar (see [specifications](https://www.bhaptics.com/shop/tactglove))

## User Manual

### (A) Turn on and connect all devices from Nodes (1), (2), (3)

### (B) Put your TactGloves on and link them to bHaptics Player - Node (3)

### (C) Open a terminal and run the tactile_ctrl.py file - Node (3)

This operation will open the communication between Node (3) and Node (2), and connect to bHaptics SDK.

````
cd ~/haptic_glove
python3 tact_glove.py <param1> <value1> <param2> <value2> <param3> <value3> ...
````

Success Indicator: Output ``Link Established. Waiting for commands from Node (2) ...``

Note: Change ``<paramX>`` and ``<valueX>`` as needed if you want to modify default application.

### (D) Source and launch ROS package for UR5e communication - Node (2)

If successfull, this operation will set the communication between Node (2) and Node (1).

````
cd ~/catkin_ws
source devel/setup.bash
roslaunch ur_robot_driver ur5e_bringup.launch robot_ip:=<insert_robot_ip> kinematics_config:=<insert_calibration_file.yaml>
````
Success Indicator: Output (...) ``Connection established`` (...)

Note: Change ``<insert_robot_ip>`` and ``<insert_calibration_file.yaml>`` as needed.

### (E) Turn robot arm on and start program grip_control.urp in Polyscope - Node (1)

This program contains the Variables Setup, the Robot Program with instructions to connect to host, and the RG Gripper initializer. It should be previously loaded via USB to the UR5e Teach Pendant containing Polyscope.

Success Indicator: Output (...) ``Robot requested program`` (...) ``Sent program to robot`` (...) ``Robot ready to receive commands``

### (F) Open other terminal and run the robot_control.py file - Node (2)

This operation will open the communication between the Tracking Operation and the Robots Movement Controllers.

````
cd ~/catkin_ws
source devel/setup.bash
rosrun haptic_u5e robot_control.py <param1> <value1> <param2> <value2> <param3> <value3> ...
````
Success Indicator: Output ``Link Established. Waiting for commands from Tracking Program ...``

Note: Change ``<paramX>`` and ``<valueX>`` as needed if you want to modify default application. See customization parameters [here](https://github.com/xriteamupv/Haptic_Teleop/tree/main/02_haptic_ur5e/src).

### (G) Open another terminal and run the arm_tracking.py file - Node (2)

This operation will activate the whole system. By default, a camera window should be displayed, which will start tracking the user's hand.

````
cd ~/catkin_ws
source devel/setup.bash
rosrun haptic_u5e arm_tracking.py <param1> <value1> <param2> <value2> <param3> <value3> ...
````

Success Indicator: Output ``System started, currently tracking your hand ...``

Note: Change ``<paramX>`` and ``<valueX>`` as needed if you want to modify default application. See customization parameters [here](https://github.com/xriteamupv/Haptic_Teleop/tree/main/02_haptic_ur5e/src).

### (H) Enjoy the experience!

![TBBT_v3](https://github.com/xriteamupv/Haptic_Teleop/assets/38531693/5eedd5d6-0d82-471a-a48e-1e65c28d1364)

