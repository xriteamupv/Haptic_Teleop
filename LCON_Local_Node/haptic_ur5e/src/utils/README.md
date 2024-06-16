## LCON Classes Description

The classes used in the programs ``robot_control.py`` and ``gripper_control.py`` are subdivided into 2 subgroups focused in the robot trajectory management and the gripper movement control respectively. This modular approach allows for further customizations and future scalability and ease-of-configuration.

### S01: Robot Arm Trajectory Management (Main: TrajectoryClient)

This subgroup of classes are reponsible for the robot trajectory management. This involves the robot controller properties specification for initialization and execution, the human arm positions received from the Tracking Intelligence, the equivalent robot arm positions, and the possible transmission of grip information to the Gripper Movement Control.

<img height="339" alt="TrajectoryClient_Updated062024" src="https://github.com/xriteamupv/Haptic_Teleop/assets/38531693/8478e2d9-564b-48a1-98d8-37fa944da45b">

<img height="339" alt="Node2_RobotControl" src="https://github.com/xriteamupv/Haptic_Teleop/assets/38531693/bb558e72-0cdc-458a-8733-57cffb84f26f">

### S02: Gripper Movement Control (Main: GripClient)

This subgroup of classes are required for the gripper actions derivations. This includes the gripper XML-RPC client specification and its synchronization with the Polyscope Thread, the gripper levels received from the Tracking Intelligence and the grip characteristics transmitted to the Haptic Enablers Conditioning.

<img height="339" alt="GripClient_Updated062024" src="https://github.com/xriteamupv/Haptic_Teleop/assets/38531693/7ea3a7ee-fe33-4de1-a829-31f82615712e">


### Static Classes

These static classes provide with functionalities specific to Coordinates Mapping, intended for using algebraic math to map different spatial domains using different arrangements, and Hand Model Configuration, intended for further optimizing the recognition algorithms.

<img width="673" alt="Static_Classes_Mapping" src="https://github.com/xriteamupv/Haptic_Teleop/assets/38531693/9594cb9a-a3bf-4e6c-9bf0-8336d61e6b42">
