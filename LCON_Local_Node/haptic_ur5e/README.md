## LCON General Description

### Functionalities

- Control of the trajectories of robot arm using coordinates (x<sub>R</sub>, y<sub>R</sub>, z<sub>R</sub>) and orientation (Quaternion(RX, RY, RZ) = (x<sub>Q</sub>, y<sub>Q</sub>, z<sub>Q</sub>)) together with gripper movements (width, force/speed) limited to their defined spatial domain.
- Object detection when current_width is different than objective_width after a preconfigured time. Grip levels (open, soft grip, medium grip, hard grip, close) configuration for different objective widths and width variations.
- Management and customizations of movement sensibilities for particular spatial ranges, and communication or processing delays for remote interactions on the local system.

### Communications

- Reception of Message from RTHN (tracking_control) related to movement tracking and basic grip instructions.
- Communication with UR robot arm via Polyscope for movement specification through RTDE and Goal-Based Cartesian Trajectory Controllers.
- Communication with OnRobot gripper via Polyscope for gripping actions through Modbus and XML Remote Procedure Call protocols.

### Models and Mappings

- *Delay Model*: Models intended for counter communication and processing delays, and make movements smoother, safer and more precise.
- *Gripper Mapper*: Mappings of grip levels to objective widths, and temporal properties to initial force/speed of the gripping action.
- *Trajectory Mapper*: Mappings of human movements to robot movements, including coordinates translation between both spatial domains.

https://github.com/xriteamupv/Haptic_Teleop/assets/38531693/36a92451-97c0-49b8-a2c8-ce360fbad63c

## Software Requirements:
- ROS Noetic v.1.16.0 (see [details](http://wiki.ros.org/noetic))
- Base OS: Ubuntu 20.04 (see [details](http://wiki.ros.org/noetic/Installation/Ubuntu))
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

### 3. Clone the repository including the LCON_Local_Node folder:
````
cd <destination_folder>
git clone https://github.com/xriteamupv/Haptic_Teleop.git
````

### 4. Keep the LCON_Local_Node folder contents and remove the rest:
````
cd <destination_folder>
mv ./Haptic_Teleop-main/LCON_Local_Node/* ~/catkin_ws/src
rm -r ./Haptic_Teleop-main
````

### 5. Install Python libraries:
Use ``pip`` to install the default Python 3 versions of the libraries.
See requirements.txt for libraries' versions.

````
pip install numpy
pip install rospy
pip install actionlib
pip install datetime
pip install pymodbus
````

### 6. Compile packages:
````
cd ~/catkin_ws
catkin_make
````
