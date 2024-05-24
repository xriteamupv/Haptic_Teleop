## NODE (2) General Description

- Capture of the position (x<sub>H</sub>, y<sub>H</sub>, z<sub>H</sub>) and movements of the human arm through a camera or tracking device.
- Gripper recognition, including how many fingers are involved in the grip.
- Message Node (1) to set new positions (x<sub>R</sub>, y<sub>R</sub>, z<sub>R</sub>, yaw, pitch, roll) to the robot arm.
- Message Node (1) to set new configurations (force, width, open/close) to the gripper.
- Message Node (3) to set new configurations (intensity, duration) to haptic actuators on the glove.

## Software Requirements:
- ROS Noetic v.1.16.0
- Base OS: Ubuntu 20.04
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
TODO: Specify

### 4. Install Python libraries:
Use ``pip`` to install the default Python 3 versions of the libraries.
See requirements.txt for libraries' versions.

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

### 5. Add haptic_teleop packages:
````
cd ~/catkin_ws/src
git clone https://github.com/xriteamupv/Haptic_Teleop.git
````

Use ``02_haptic_ur5e`` package. Remove the rest.

### 6. Compile packages - Node (2)
````
cd ~/catkin_ws
catkin_make
````
