## NODE (2) Classes Description

The classes used in the programs ``tracking_control.py``, ``robot_control.py`` and ``haptic_control.py`` are subdivided into 2 subgroups focused in the tracking intelligence, the trajectory management and the haptic enablers conditioning respectively. This modular approach allows for further customizations and future scalability and ease-of-configuration.

### S01: Tracking Intelligence (Main: HandTracker)

This subgroup of classes are responsible for the human movement tracking and hand gestures recognition. This involves the tracking algorithms specification for model configuration, the hand landmarks visualization for user feedback, the system state for action registries, and the pre-processed keypoints history for movement predictions.

<img height="339" alt="Diagrama_Arm_Tracking" src="https://github.com/xriteamupv/Haptic_Teleop/assets/38531693/b0721a8f-1492-43f7-91eb-9ce13c57ce72">
<img height="339" alt="Node2_TrackingControl_V2" src="https://github.com/xriteamupv/Haptic_Teleop/assets/38531693/37564d44-b66a-4176-a88a-a7ea551e6b75">


### S02: Trajectory Management (Main: TrajectoryClient)

This subgroup of classes are reponsible for the robot trajectory management and gripper actions derivations. This involves the robot controller properties specification for initialization and execution, the human arm positions received from the Tracking Intelligence, the equivalent robot arm positions and the gripper information registry and delineation.

<img height="339" alt="Diagrama_Robot_Control" src="https://github.com/xriteamupv/Haptic_Teleop/assets/38531693/4c406cb4-09da-4473-ab33-b3f991b2fd4d">
<img height="339" alt="Node2_RobotControl" src="https://github.com/xriteamupv/Haptic_Teleop/assets/38531693/bb558e72-0cdc-458a-8733-57cffb84f26f">

### S03: Haptic Enablers Conditioning (Main: HapticsClient)

This subgroup of classes are responsible for the conditioning of haptic enablers received from the Tracking Intelligence and the Trajectory Management, intended to provide the Haptic Feedback node with pre-analyzed characteristics (i.e. vibration intensity and duration) and configurations (i.e. actuator/s to activate) for the tactile gloves.

TODO: Add UML Diagram with classes.
<img height="339" alt="Node2_HapticControl" src="https://github.com/xriteamupv/Haptic_Teleop/assets/38531693/223c5ac7-d864-4a60-b634-716d2e4097b2">

### Static Classes

These static classes provide with functionalities specific to Coordinates Mapping, intended for using algebraic math to map different spatial domains using different arrangements, and Hand Model Configuration, intended for further optimizing the recognition algorithms.

<img width="673" alt="Static_Classes_Mapping" src="https://github.com/xriteamupv/Haptic_Teleop/assets/38531693/9594cb9a-a3bf-4e6c-9bf0-8336d61e6b42">
