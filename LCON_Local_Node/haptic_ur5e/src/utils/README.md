## LCON Classes Description

The classes used in the programs ``robot_control.py`` and ``gripper_control.py`` are subdivided into 2 subgroups focused in the robot trajectory management and the gripper movement control respectively. This modular approach allows for further customizations and future scalability and ease-of-configuration.

### S01: Robot Arm Trajectory Management (Main: TrajectoryClient)

This subgroup of classes are reponsible for the robot trajectory management. This involves the robot controller properties specification for initialization and execution, the human arm positions received from the Tracking Intelligence, the equivalent robot arm positions, and the possible transmission of grip information to the Gripper Movement Control.

<img height="339" alt="TrajectoryClient_Updated062024" src="https://github.com/xriteamupv/Haptic_Teleop/assets/38531693/8478e2d9-564b-48a1-98d8-37fa944da45b">

### S02: Gripper Movement Control (Main: GripClient)

This subgroup of classes are required for the gripper actions derivations. This includes the gripper XML-RPC client specification and its synchronization with the Polyscope Thread, the gripper levels received from the Tracking Intelligence and the grip characteristics transmitted to the Haptic Enablers Conditioning.

<img height="379" alt="GripClient_Updated062024" src="https://github.com/xriteamupv/Haptic_Teleop/assets/38531693/7ea3a7ee-fe33-4de1-a829-31f82615712e">


### Static Classes

These static classes provide with functionalities specific to Coordinates Mapping, intended for using algebraic math to map different spatial domains using different arrangements.

<img width="600" alt="FigUMLCharts_Trajectory" src="https://github.com/xriteamupv/Haptic_Teleop/assets/38531693/880b5396-2ff7-4e61-851b-ed67eb197649">

<img width="610" alt="FigUMLCharts_Width" src="https://github.com/xriteamupv/Haptic_Teleop/assets/38531693/32a81790-e9ad-4b1f-a7fa-a6c3405c5e3d">

<img width="600" alt="FigUMLCharts_Force" src="https://github.com/xriteamupv/Haptic_Teleop/assets/38531693/b513365b-a3c5-4063-8574-bd0d5fba830f">
