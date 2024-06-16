## LCON Programs Description

### S01: robot_control.py using TrajectoryClient

This program manages the communication with Polyscope, the robot arm movement characteristics using Pose-based/Joint-based/Forward Cartesian Trajectory Controllers, and the gripper actions using Modbus through ROS. It also maps the coordinates from the tracking spatial domain to the robot spatial domain, as well as configures several customization options related to the interaction with the robots.

#### Customization Parameters:

``--communication_mode <int>``: Communication flows as specified [here](https://github.com/xriteamupv/Haptic_Teleop/tree/main/comms).
- ``--mapping <int 0-3>``: Mapping algorithm (default: 0, i.e. linear_mapping), see subsection below for specifications.
- ``--controller <int 0-2>``: Goal-based Cartesian Controller: Pose-based (0), Joint-based (1), Forward trajectory controllers (2). Default: 0.
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
- ``--bidirectional <bool True/False>``: Enable bidirectional communication with tracking_control.py and gripper_control.py.

### S02: gripper_control.py using HapticsClient

This program controls the grip movements and robot communications according to the tracking characteristics, including opening/closing the gripper to an objective width (1D position), detecting possible objects begin gripped and informing the Remote Node for haptic feedback. It also regulates the strength that needs to be applied to the grip movement, which determines its duration and open/close force.

- ``--communication_mode <int>``: Communication flows as specified [here](https://github.com/xriteamupv/Haptic_Teleop/tree/main/comms).
- ``--max_width <float>``: Maximum width allowed for the gripper. This constraints the gripper range of movement. Default 100.
- ``--max_force <float>``: Maximum force allowed for the gripper. This constraints the gripper range of speed. Default 40.
- ``--start_width <float>``: Initial width when starting the gripper_control program. It can be more than max_width. Default 100.
- ``--width_tolerance <float>``: Deadband for gripper width. The gripper won't do movements less than this width variation. Default 2.
- ``--force_tolerance <float>``: Deadband for gripper force. The gripper won't do movements less than this force variation. Default 2.
- ``--max_duration <float>``: Default 60.
- ``--time_max_width <float>``: Default 5.
- ``--static_force <float>``: Unique constant force value used in the case of standard non-variante force model. Default 39 (Newtons).
- ``--width_model <int>``: Model used for mapping Grip Levels to Objective Width, such as Fixed by Levels (0), Linear Regulation (1), and GripperMapper models (2-5). Default 0.
- ``--force_model <int>``: Model used for mapping Temporal Conditions to Initial Force, such as Static Force (0), Dynamic Force (1), and GripperMapper models (2-5). Default 0.
- ``--delay_model <int>``: Model used for varying the delay between the application of successive instructions to the robot, such as Static Delay (0), Linearly Variant Delay (1) and DelayModel methods (2-5). Default 1.
- ``--grip_levels <int>``: Maximum amount of grip levels to consider (open, soft, medium, hard, close). Default 4.
- ``--bidirectional <bool True/False>``: Enable bidirectional communication with robot_control.py and haptic_control.py.

parser.add_argument( "--communication_mode", type = int, default = 0 )
parser.add_argument( "--max_width", type = float, default = 100.0 )
parser.add_argument( "--max_force", type = float, default = 40.0 )
parser.add_argument( "--start_width", type = float, default = 100.0 )
parser.add_argument( "--width_tolerance", type = float, default = 2.0 )
parser.add_argument( "--force_tolerance", type = float, default = 2.0 )
parser.add_argument( "--max_duration", type = float, default = 60.0 ) # mseconds
parser.add_argument( "--time_max_width", type = float, default = 5.0 ) # seconds
parser.add_argument( "--static_force", type = float, default = 39.0 ) # Netwons
parser.add_argument( "--width_model", type = int, default = 0 )
parser.add_argument( "--force_model", type = int, default = 0 )
parser.add_argument( "--delay_model", type = int, default = 1 )
parser.add_argument( "--grip_levels", type = int, default = 4 ) # CHECK 3 or 4
parser.add_argument( "--bidirectional", type = int, default = 0 ) #PENDING

### Static Classes for Coordinates Mapping and Hand Models Configuration

These classes provide additional functionalities for several types of mappings of (x<sub>H</sub>, y<sub>H</sub>, z<sub>H</sub>) to (x<sub>R</sub>, y<sub>R</sub>, z<sub>R</sub>).

By default, note that x<sub>H</sub> = - x<sub>R</sub>; y<sub>H</sub> = - z<sub>R</sub>; z<sub>H</sub> =  y<sub>R</sub>. Also note that the position ranges for each direction differ between spatial domains.

<img src="../../images/Camera_X-axis.gif" width="320"/> <img src="../../images/Camera_Y-axis.gif" width="320"/> <img src="../../images/Camera_Z-axis.gif" width="320"/>

<img src="../../images/Robot_X-axis2.gif" width="320"/> <img src="../../images/Robot_Z-axis2.gif" width="320"/> <img src="../../images/Robot_Y-axis2.gif" width="320"/>
