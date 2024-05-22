# Haptic Teleoperation Experience

Video AI-based Teleoperation of UR5e Robot Arm and OnRobot RG2 Gripper using ROS packages and complemented with Haptic Feedback from bHaptics TactGloves

## Project Description

The present project aims to develop an immersive teleoperation system for the remote contorl of a robotic arm. The system allows a user to manipulate remote objects imitating the movements of their own arm through movement capture sensors. The experience is enriched by haptic feedback, providing the user with a real contact sensation with the manipulated object.

<img width="1257" alt="DiagramaBloques_TFM" src="https://github.com/xriteamupv/Haptic_Teleop/assets/38531693/9e5c0d8f-71b3-4309-9508-f7687ed8bd68">

### The system is composed of three main nodes:

#### (1) Robotic arm and grip clamp controller node:

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
- ROS Noetic 1.16.0
- Python 3.8.3 (Python 3)

### Software for Node (3):
- bHaptics Player
- Python 3.8.3 (Python 3)

## Installation:

### 1. Create workspace:
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

Node (3) -> Use ``tact-glove`` package. Remove the rest.

### 6. Compile packages - Node (2)
````
cd ~/catkin_ws
catkin_make
````

## User Manual

TODO

## Program Description

TODO

![DiagramaUML_TFMv2](https://github.com/xriteamupv/Haptic_Teleop/assets/38531693/f62a2cd5-4789-4598-8863-8f386493930a)

TODO
