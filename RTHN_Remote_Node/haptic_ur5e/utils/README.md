## RTHN Classes Description

The classes used in the programs ``tracking_control.py`` and ``haptic_control.py`` are subdivided into 2 subgroups focused in the tracking intelligence and the haptic enablers conditioning respectively. This modular approach allows for further customizations and future scalability and ease-of-configuration.

### S01: Tracking Intelligence (Main: HandTracker)

This subgroup of classes are responsible for the human movement tracking and hand gestures recognition. This involves the tracking algorithms specification for model configuration, the hand landmarks visualization for user feedback, the system state for action registries, and the pre-processed keypoints history for movement predictions.

<img height="379" alt="HandTracker_Updated062024" src="https://github.com/xriteamupv/Haptic_Teleop/assets/38531693/5f4d374c-9eb5-40b3-8a26-0d725571b952">


### S02: Haptic Enablers Conditioning (Main: HapticClient)

This subgroup of classes are responsible for the conditioning of haptic enablers received from the Tracking Intelligence and the Trajectory Management, intended to provide the Haptic Feedback node with pre-analyzed characteristics (i.e. vibration intensity and duration) and configurations (i.e. actuator/s to activate) for the tactile gloves.

<img height="379" alt="HapticClient_Updated062024" src="https://github.com/xriteamupv/Haptic_Teleop/assets/38531693/5d271f92-e3dc-45b2-9c62-2be7ad057dcf">


### Static Classes

These static classes provide with functionalities specific to Coordinates Mapping, intended for using algebraic math to map different spatial domains using different arrangements, and Hand Model Configuration, intended for further optimizing the recognition algorithms.

<img width="600" alt="FigUMLStatic_RTHN" src="https://github.com/xriteamupv/Haptic_Teleop/assets/38531693/fef2026a-01ed-42fd-b8cf-811062490ad7">

