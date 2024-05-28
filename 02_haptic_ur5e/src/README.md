## NODE (2) Programs Description

### S01: arm_tracking.py using HandTracker

This program captures the human hand position and movements, while it recognizes pre-trained hand gestures such as 'OK' to activate robot control, or 'Grip 2F/3F/4F/5F' to detect the human grip with different amounts of fingers. In order to activate the robot control, the hand needs to be placed within an initial zone radius which can be configured together with other several customization option related to the tracking procedure.

#### Customization Parameters:

- ``--device <int>``: Camera device identifier (default: 0, i.e. integrated camera).
- ``--width <int>``: Video width as quantity of pixels (default: 960).
- ``--height <int>``: Video height as quantity of pixels (default: 540).
- ``--initial <int 0-8>``: Initial position to enable tracking: up_left, up_center, up_right, ..., low_right (default: 4, i.e. mid_center).
- ``--tracking <int 0-2>``: Hand landmarks filter: entire hand, palm+index+thumb, palm (default: 0, i.e. entire hand)
- ``--use_static_image_mode``: Enables Hand Detection to run every input image (default: treat the input images as video stream).
- ``--min_detection_confidence <float 0.0-1.0>``: Minimum detection confidence for gesture recognition (default 0.7).
- ``--min_tracking_confidence <float 0.0-1.0>``: Minimum tracking confidence for movement precision (default 0.5).
- ``--record <str_video_name>``: Enable MP4 video recording and specify the video filename (default: None, i.e. not recording).
- ``--with_orientation <bool True/False>``: Enable Tracking Hand Orientation as (yaw, pitch, roll) perpendicular to palm.
- ``--bidirectional_comms <bool True/False>``: Enable bidirectional communication with robot_control.py and haptic_control.py.

<img src="../../images/Camera_Tracking0.gif" width="300"/> <img src="../../images/Camera_Tracking1.gif" width="300"/> <img src="../../images/Camera_Tracking2.gif" width="300"/>

### S02: robot_control.py using TrajectoryClient

This program manages the communication with Node (1), the robot movement characteristics using Pose-based/Joint-based/Forward Cartesian Trajectory Controllers, and the gripper actions using Modbus through ROS. It also maps the coordinates from the tracking spatial domain to the robot spatial domain, as well as configures several customization options related to the interaction with the robots.

#### Customization Parameters:

- ``--mapping <int 0-3>``: Mapping algorithm (default: 0, i.e. linear_mapping), see subsection below for specifications.
- ``--controller <int 0-2>``: Goal-based Cartesian Controller: (pose-based/joint-based/forward
- ``--initial <int 0-8>``: Initial position of robot arm: up_left, up_center, up_right, ..., low_right (default: 4, i.e. mid_center).
- ``--oneaxis <int 0-2>``: Enable the robot arm to move in only 1 axis: x, y, z (default: None, i.e. movement in all 3 axis). 
- ``--twoaxis <int>``: Enable the robot arm to move in only 2 axis: xy, yz, xz (default: None, i.e. movement in all 3 axis). 
- ``--duration <int>``: Movement duration configuration: fixed-value (0), dynamic mirroring (1), linear position-based (2). (default: 2)
- ``--velocity <int 0-2>``: Movement velocity configuration: self-adjusting (0), fixed value (1), linear dynamic (2). (default 0)
- ``--acceleration <int>``: Movement acceleration configuration: self-adjusting (0), fixed value (1), linear dynamic (2). (default 0)
- ``--precision <float>``: Movement tolerance, avoiding trajectories with an euclidean distance less than provided value. (default: 0.0)
- ``--wait_time <float>``: Waiting time in milliseconds for the reception of new commands. (default: None, i.e. self-adjusting)
- ``--mimic <bool True/False>``: Order the robot arm to mimic the user's movements (default: False, i.e. mirror user's movement).
- ``--inverted <bool True/False>``: Order the robot to invert only the up-down movement (default: False, i.e. no inversion).
- ``--bidirectional <bool True/False>``: Enable bidirectional communication with arm_tracking.py and haptic_control.py.

TODO: Add illustrative GIFs.

### S03: haptic_control.py using HapticsClient

TODO: Add Description.

TODO: Specify Customizations. Add illustrative GIFs.

### Static Classes for Coordinates Mapping and Hand Models Configuration

These classes provide additional functionalities for several types of mappings of (x<sub>H</sub>, y<sub>H</sub>, z<sub>H</sub>) to (x<sub>R</sub>, y<sub>R</sub>, z<sub>R</sub>), and for training new gestures or enhance pre-trained existing recognitions of the hand detection's neural network. 

By default, note that x<sub>H</sub> = - x<sub>R</sub>; y<sub>H</sub> = - z<sub>R</sub>; z<sub>H</sub> =  y<sub>R</sub>. Also note that the position ranges for each direction differ between spatial domains.

<img src="../../images/Camera_X-axis.gif" width="300"/> <img src="../../images/Camera_Y-axis.gif" width="300"/> <img src="../../images/Camera_Z-axis.gif" width="300"/>

<img src="../../images/Robot_X-axis2.gif" width="300"/> <img src="../../images/Robot_Z-axis2.gif" width="300"/> <img src="../../images/Robot_Y-axis2.gif" width="300"/>
