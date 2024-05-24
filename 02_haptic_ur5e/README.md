## NODE (2) Programs Description

- Capture of the position (x<sub>H</sub>, y<sub>H</sub>, z<sub>H</sub>) and movements of the human arm through a camera or tracking device.
- Gripper recognition, including how many fingers are involved in the grip.
- Message Node (1) to set new positions (x<sub>R</sub>, y<sub>R</sub>, z<sub>R</sub>, yaw, pitch, roll) to the robot arm.
- Message Node (1) to set new configurations (force, width, open/close) to the gripper.
- Message Node (3) to set new configurations (intensity, duration) to haptic actuators on the glove.

### arm_tracking.py using HandTracker

This program captures the human hand position and movements, while it recognizes pre-trained hand gestures such as 'OK' to activate robot control, or 'Grip 2F/3F/4F/5F' to detect the human grip with different amounts of fingers. In order to activate the robot control, the hand needs to be placed within an initial zone radius which can be configured together with other several customization option related to the tracking procedure .

<img width="439" alt="Diagrama_Arm_Tracking" src="https://github.com/xriteamupv/Haptic_Teleop/assets/38531693/b0721a8f-1492-43f7-91eb-9ce13c57ce72">

TODO: Specify Customizations. Add illustrative GIFs.

### robot_control.py using TrajectoryClient

This program manages the communication with Node (1), the robot movement characteristics using Pose-based/Joint-based/Forward Cartesian Trajectory Controllers, and the gripper actions using Modbus through ROS. It also maps the coordinates from the tracking spatial domain to the robot spatial domain, as well as configures several customization options related to the interaction with the robots.

<img width="510" alt="Diagrama_Robot_Control" src="https://github.com/xriteamupv/Haptic_Teleop/assets/38531693/4c406cb4-09da-4473-ab33-b3f991b2fd4d">

TODO: Specify Customizations. Add illustrative GIFs.

### Static Classes for Coordinates Mapping and Hand Models Configuration

These classes provide additional functionalities for several types of mappings of (x<sub>H</sub>, y<sub>H</sub>, z<sub>H</sub>) to (x<sub>R</sub>, y<sub>R</sub>, z<sub>R</sub>), and for training new gestures or enhance pre-trained existing recognitions of the hand detection's neural network. 

By default, note that x<sub>H</sub> = - x<sub>R</sub>; y<sub>H</sub> = - z<sub>R</sub>; z<sub>H</sub> =  y<sub>R</sub>. Also note that the position ranges for each direction differ between spatial domains.

<img width="673" alt="Static_Classes_Mapping" src="https://github.com/xriteamupv/Haptic_Teleop/assets/38531693/9594cb9a-a3bf-4e6c-9bf0-8336d61e6b42">

TODO: Specify Customizations. Add illustrative GIFs.
