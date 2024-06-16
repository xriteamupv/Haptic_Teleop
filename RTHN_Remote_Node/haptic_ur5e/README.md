## NODE (2) Programs Description

### S01: tracking_control.py using HandTracker

This program captures the human hand position and movements, while it recognizes pre-trained hand gestures such as 'OK' to activate robot control, or 'Grip 2F/3F/4F/5F' to detect the human grip with different amounts of fingers. In order to activate the robot control, the hand needs to be placed within an initial zone radius which can be configured together with other several customization option related to the tracking procedure.

#### Customization Parameters:

- ``--communication_mode <int>``: Communication flows as specified [here](https://github.com/xriteamupv/Haptic_Teleop/tree/main/comms).
- ``--device <int>``: Camera device identifier (default: 0, i.e. integrated camera).
- ``--width <int>``: Video width as quantity of pixels (default: 960).
- ``--height <int>``: Video height as quantity of pixels (default: 540).
- ``--initial <int 0-8>``: Initial position to enable tracking: up_left, up_center, up_right, ..., low_right. (Default: 4, i.e. mid_center).
- ``--tolerance <int 0-2>``: Tolerance for establishing initial position: low (0), medium (1), high (2) tolerance. (Default: 1).
- ``--tracking <int 0-2>``: Hand landmarks filter: entire hand (0), palm+index+thumb (1), palm (2). (Default: 0, i.e. entire hand).
- ``--gloves_color <int 0-3>``: Hands or Gloves color: no gloves (0), blue (1), yellow (2), black (3). (Default: 1, i.e. blue gloves)
- ``--use_static_image_mode``: Enables Hand Detection to run every input image. (Default: treat the input images as video stream).
- ``--min_detection_confidence <float 0.0-1.0>``: Minimum detection confidence for gesture recognition. (Default 0.7).
- ``--min_tracking_confidence <float 0.0-1.0>``: Minimum tracking confidence for movement precision. (Default 0.5).
- ``--record <str_video_name>``: Enable MP4 video recording and specify the video filename. (Default: None, i.e. not recording).
- ``--with_orientation <bool True/False>``: Enable Tracking Hand Orientation as (yaw, pitch, roll) perpendicular to palm.
- ``--bidirectional_comms <bool True/False>``: Enable bidirectional communication with robot_control.py and haptic_control.py.

<img src="../../images/Camera_Tracking0.gif" width="320"/> <img src="../../images/Camera_Tracking1.gif" width="320"/> <img src="../../images/Camera_Tracking2.gif" width="320"/>

### S03: haptic_control.py using HapticsClient

This program processes grip output characteristics and sends it to the haptic glove that is being used. The required grip outputs are width variation (i.e. difference between objective_width and current_width in millimeters), initial gripping force (Newtons 0.0-100.0), amount of fingers used (2-6), and a timestamp with the grip occurance time. The required haptic inputs are actuators indexes (0-5), sensation intensity (percentage 0.0-100.0), and stimuli duration in milliseconds.

#### Customization Parameters:

- ``--communication_mode <int>``: Communication flows as specified [here](https://github.com/xriteamupv/Haptic_Teleop/tree/main/comms).
- ``--max_fingers <int>``: Default: 6 (5F+Palm).
- ``--min_intensity <float>``: Default: 10.
- ``--max_intensity <float>``: Default: 100.
- ``--precision_duration <float>``: Default: 100.
- ``--precision_width <float>``: Default: 3.3.
- ``--initial_delay <float>``: Default: 0.
- ``--final_delay <float>``: Default: 0.
- ``--width_weight <float>``: Default: 0.7.
- ``--width_threshold <float>``: Default: 70.
- ``--intensity_shift <float>``: Default: 0.
- ``--static_intensity <float>``: Default: 80.
- ``--intensity_model <int>``: Default: 0.
- ``--right_hand_enabled <bool True/False>``: Default: True.
- ``--discrete_sensations <bool True/False>``: Default: True.
- ``--bidirectional_comms <bool True/False>``: Default: False.

TODO: Add illustrative GIFs.

### Static Classes for Coordinates Mapping and Hand Models Configuration

These classes provide additional functionalities for several types of mappings of (x<sub>H</sub>, y<sub>H</sub>, z<sub>H</sub>) to (x<sub>R</sub>, y<sub>R</sub>, z<sub>R</sub>), and for training new gestures or enhance pre-trained existing recognitions of the hand detection's neural network. 

By default, note that x<sub>H</sub> = - x<sub>R</sub>; y<sub>H</sub> = - z<sub>R</sub>; z<sub>H</sub> =  y<sub>R</sub>. Also note that the position ranges for each direction differ between spatial domains.

<img src="../../images/Camera_X-axis.gif" width="320"/> <img src="../../images/Camera_Y-axis.gif" width="320"/> <img src="../../images/Camera_Z-axis.gif" width="320"/>

<img src="../../images/Robot_X-axis2.gif" width="320"/> <img src="../../images/Robot_Z-axis2.gif" width="320"/> <img src="../../images/Robot_Y-axis2.gif" width="320"/>
