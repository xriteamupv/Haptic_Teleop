## RTHN General Description

### Functionalities:

- Capture of the position (x<sub>H</sub>, y<sub>H</sub>, z<sub>H</sub>) and movements of the human arm through a video camera or tracking device.
- Recognize different gripper configurations, including types of grips (soft, medium, hard) and how many fingers (2F, 3F, 4F, 5F) are involved in the grip.
- Management of complex haptic sensations by setting new configurations (vibration intensity, stimulus duration) to each haptic actuator on the glove.

### Communications:
- Message LCON to set new positions (x<sub>R</sub>, y<sub>R</sub>, z<sub>R</sub>, yaw, pitch, roll) to the robot arm.
- Message LCON to set new configurations (initial force, objective width, open/close) to the robot gripper.
- Receive updated gripper configurations (gripping force, width variation, timestamp) from LCON for haptic glove.

### Models and Mappings:
- *Hand Model*: Deep-learning training model and customizations for hand detection and tracking.
- *Gesture Model*: Deep-learning training model and customizations for gesture recognition and additions.
- *Haptic Mappers*: Mathematical models that map gripper outputs (width variation and grip force) to haptic inputs (vibration intensity). 

TODO: AGREGAR VIDEO CON GUANTES

## Software Requirements:
- bHaptics Player (see [details](https://www.bhaptics.com/software/player/))
- Base OS: Windows 10 (see [details](https://www.microsoft.com/en-gb/software-download/windows10ISO))
- Python 3.11.3 (Python 3)

## Installation:

### 1. Clone the repository including the RTHN_Remote_Node folder:
````
cd <destination_folder>
git clone https://github.com/xriteamupv/Haptic_Teleop.git
````

### 2. Keep the RTHN_Remote_Node folder contents and remove the rest:
````
move -y ./RTHN_Remote_Node/* ./
rm -r ./RTHN_Remote_Node
rm -r ./LCON_Local_Node
rm -r ./Teloperation_Local_Node
````

### 3. Modify the following files with your corresponding addresses:
````
./haptic_ur5e/src/utils/Classifier.py
./haptic_ur5e/src/model/keypoint_classifier/keypoint_classifier.py
./haptic_ur5e/src/model/point_history_classifier/point_history_classifier.py
````

If you decide to use Windows Subsystem for Linux (WSL), you have to modify the addresses of:
````
./init_remote_node.sh
./initialize_remote_node.sh
````

### 4. Install Python libraries:
Use ``pip`` to install the default Python 3 versions of the libraries.
See requirements.txt for libraries' versions.

````
pip install numpy
pip install datetime
pip install pymodbus
pip install websocket
pip install websocket-client
pip install opencv-python
pip install mediapipe
pip install tensorflow
````
