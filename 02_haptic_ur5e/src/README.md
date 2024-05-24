## NODE (2) Programs Dsecription

### arm_tracking.py using HandTracker

This program captures the human hand position and movements, while it recognizes pre-trained hand gestures such as 'OK' to activate robot control, or 'Grip 2F/3F/4F/5F' to detect the human grip with different amounts of fingers. In order to activate the robot control, the hand needs to be placed within an initial zone radius which can be configured together with other several customization option related to the tracking procedure.

TODO: Specify Customizations. Add illustrative GIFs.

### robot_control.py using TrajectoryClient

This program manages the communication with Node (1), the robot movement characteristics using Pose-based/Joint-based/Forward Cartesian Trajectory Controllers, and the gripper actions using Modbus through ROS. It also maps the coordinates from the tracking spatial domain to the robot spatial domain, as well as configures several customization options related to the interaction with the robots.

### haptic_control.py using HapticsClient

TODO: Add Description.

TODO: Specify Customizations. Add illustrative GIFs.

### Static Classes for Coordinates Mapping and Hand Models Configuration

These classes provide additional functionalities for several types of mappings of (x<sub>H</sub>, y<sub>H</sub>, z<sub>H</sub>) to (x<sub>R</sub>, y<sub>R</sub>, z<sub>R</sub>), and for training new gestures or enhance pre-trained existing recognitions of the hand detection's neural network. 

By default, note that x<sub>H</sub> = - x<sub>R</sub>; y<sub>H</sub> = - z<sub>R</sub>; z<sub>H</sub> =  y<sub>R</sub>. Also note that the position ranges for each direction differ between spatial domains.

TODO: Specify Customizations. Add illustrative GIFs.
