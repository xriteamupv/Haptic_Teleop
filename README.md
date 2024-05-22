# Haptic Teleoperation Experience

Video AI-based Teleoperation of UR5e and OnRobot RG2 Gripper using ROS packages and complemented with Haptic Feedback from BHaptics gloves

The present project aims to develop an immersive teleoperation system for the remote contorl of a robotic arm. The system allows a user to manipulate remote objects imitating the movements of their own arm through movement capture sensors. The experience is enriched by haptic feedback, providing the user with a real contact sensation with the manipulated object.

<img width="1257" alt="DiagramaBloques_TFM" src="https://github.com/xriteamupv/Haptic_Teleop/assets/38531693/9e5c0d8f-71b3-4309-9508-f7687ed8bd68">

The system is composed of three main nodes:

## (1) Robotic arm and grip clamp controller node:

This node houses the UR5e robotic arm controller, the OnRobot RG2 clamp and the Polyscope communications interface. It is responsible for transmitting information from states (arm position, trajectories result, grip amplitude and strength), as well as receiving the movement instructions from the movement capture node and sending them to the robotic arm for their execution.

## (2) Movement capture node: 

This node captures the movements of the human arm through a camera and the Mediapipe library, or a tracking device. The movement data is processed and sent to the master controller node, indicating the robotic arm how to move using ROS and the rospy library to imitate user movements.

## (3) Haptic feedback node:

This node receives the strength information of the RG2 clamp of the robotic arm together with the result of the grip recognition. The information is processed and sent to the bHaptics tactile gloves using bHaptics Player and the tact-python library, providing the user with a sensory vibro-tactile feedback when the robotic arm contacts the manipulated object.

![DiagramaUML_TFMv2](https://github.com/xriteamupv/Haptic_Teleop/assets/38531693/f62a2cd5-4789-4598-8863-8f386493930a)


