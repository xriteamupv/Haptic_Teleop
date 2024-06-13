#!/bin/bash

############################################################
# Help                                                     #
############################################################
Help()
{
    # Display Help
    echo "--device <int>: Camera device identifier (default: 0, i.e. integrated camera)."
    echo "--width <int>: Video width as quantity of pixels (default: 960)."
    echo "--height <int>: Video height as quantity of pixels (default: 540)."
    echo "--initial <int 0-8>: Initial position to enable tracking: up_left, up_center, up_right, ..., low_right. (Default: 4, i.e. mid_center)."
    echo "--tolerance <int 0-2>: Tolerance for establishing initial position: low (0), medium (1), high (2) tolerance. (Default: 1)."
    echo "--tracking <int 0-2>: Hand landmarks filter: entire hand (0), palm+index+thumb (1), palm (2). (Default: 0, i.e. entire hand)."
    echo "--gloves_color <int 0-3>: Hands or Gloves color: no gloves (0), blue (1), yellow (2), black (3). (Default: 1, i.e. blue gloves)"
    echo "--use_static_image_mode: Enables Hand Detection to run every input image. (Default: treat the input images as video stream)."
    echo "--min_detection_confidence <float 0.0-1.0>: Minimum detection confidence for gesture recognition. (Default 0.7)."
    echo "--min_tracking_confidence <float 0.0-1.0>: Minimum tracking confidence for movement precision. (Default 0.5)."
    echo "--record <str_video_name>: Enable MP4 video recording and specify the video filename. (Default: None, i.e. not recording)."
    echo "--with_orientation <bool True/False>: Enable Tracking Hand Orientation as (yaw, pitch, roll) perpendicular to palm."
    echo "--bidirectional_comms <bool True/False>: Enable bidirectional communication with robot_control.py and haptic_control.py."
    echo
}

############################################################
# Configurations                                           #
############################################################

DeviceOptions()
{
    if [[ "$device" == "0" ]]; then
        echo "DEVICE: Integrated Camera (default: $device)"
    else
        echo "DEVICE: Custom Video Input ($device)"
    fi
}

InitialPositionOptions()
{
    if [[ "$initial" == "0" ]]; then
        echo "INITIAL POSITION: Upper-Left ($initial)"
    elif [[ "$initial" == "1" ]]; then
        echo "INITIAL POSITION: Upper-Center ($initial)"
    elif [[ "$initial" == "2" ]]; then
        echo "INITIAL POSITION: Upper-Right ($initial)"
    elif [[ "$initial" == "3" ]]; then
        echo "INITIAL POSITION: Middle-Left ($initial)"
    elif [[ "$initial" == "5" ]]; then
        echo "INITIAL POSITION: Middle-Right ($initial)"
    elif [[ "$initial" == "6" ]]; then
        echo "INITIAL POSITION: Lower-Left ($initial)"
    elif [[ "$initial" == "7" ]]; then
        echo "INITIAL POSITION: Lower-Center ($initial)"
    elif [[ "$initial" == "8" ]]; then
        echo "INITIAL POSITION: Lower-Right ($initial)"
    else
        initial=4
        echo "INITIAL POSITION: Middle-Center (default: $initial)"
    fi
}

ToleranceOptions()
{
    if [[ "$tolerance" == "0" ]]; then
        echo "TOLERANCE: Lowest ($tolerance)"
    elif [[ "$tolerance" == "2" ]]; then
        echo "TOLERANCE: Highest ($tolerance)"
    else
        tolerance=1
        echo "TOLERANCE: Medium (default: $tolerance)"
    fi
}

TrackingOptions()
{
    if [[ "$tracking" == "1" ]]; then
        echo "TRACKING: Palm + 2F ($tracking)"
    elif [[ "$tracking" == "2" ]]; then
        echo "TRACKING: Palm ($tracking)"
    else
        tracking=0
        echo "TRACKING: Palm + 5F (default: $tracking)"
    fi
}

GlovesColorOptions()
{
    if [[ "$color" == "0" ]]; then
        echo "GLOVES_COLOR: No Gloves ($color)"
    elif [[ "$color" == "2" ]]; then
        echo "GLOVES_COLOR: Black Gloves ($color)"
    elif [[ "$color" == "3" ]]; then
        echo "GLOVES_COLOR: Yellow Gloves ($color)"
    else
        color=1
        echo "GLOVES_COLOR: Blue Gloves (default: $color)"
    fi
}

RecordOptions()
{
    if [[ "$record" == "" ]]; then
        echo "RECORD: No recording (default)"
    else
        echo "RECORD: Enabled ($record.mp4)"
    fi
}

SummaryTracking()
{
    # Output the values
    echo ""
    echo "===============  TRACKER CONTROL  ================="
    DeviceOptions
    echo "WIDTH: $width pixels"
    echo "HEIGHT: $height pixels"
    InitialPositionOptions
    ToleranceOptions
    TrackingOptions
    GlovesColorOptions
    echo "STATIC_IMAGE_MODE: $use_static_image_mode"
    echo "MIN_DETECTION_CONFIDENCE: $min_detection_confidence/1.0"
    echo "MIN_TRACKING_CONFIDENCE: $min_tracking_confidence/1.0"
    RecordOptions
    echo "WITH_ORIENTATION: $with_orientation"
    echo "BIDIRECTIONAL_COMMS: $bidirectional_tracking"
    echo "====================================================="
    echo ""
}

MaxFingersOptions()
{
    if [[ "$max_fingers" == "0" ]]; then
        echo "FINGERS: NO ($max_fingers)"
    elif [[ "$max_fingers" == "1" ]]; then
        echo "FINGERS: Thumb ($max_fingers)"
    elif [[ "$max_fingers" == "2" ]]; then
        echo "FINGERS: Thumb + Index ($max_fingers)"
    elif [[ "$max_fingers" == "3" ]]; then
        echo "FINGERS: Thumb + Index + Middle ($max_fingers)"
    elif [[ "$max_fingers" == "4" ]]; then
        echo "FINGERS: Thumb + Index + Middle + Anular ($max_fingers)"
    else
        max_fingers=5
        echo "FINGERS: Thumb + Index + Middle + Anular + Little (default: $max_fingers)"
    fi
}

IntensityModelOptions()
{
    if [[ "$intensity_model" == "1" ]]; then
        echo "INTENSITY_MODEL: Linear Interpolation ($intensity_model)"
    elif [[ "$intensity_model" == "2" ]]; then
        echo "INTENSITY_MODEL: Weighted Interpolation ($intensity_model)"
    elif [[ "$intensity_model" == "3" ]]; then
        echo "INTENSITY_MODEL: Dynamic Weighted Interpolation ($intensity_model)"
    elif [[ "$intensity_model" == "4" ]]; then
        echo "INTENSITY_MODEL: Normalized Biased Weighted Interpolation ($intensity_model)"
    else
        intensity_model=0
        echo "INTENSITY_MODEL: Static Specification (default: $intensity_model)"
    fi
}

HandEnabledOptions()
{
    if [[ "$right_hand_enabled" == "False" ]]; then
        echo "HAND ENABLED: Left (please use Left Glove)"
    else
        right_hand_enabled="True"
        echo "HAND ENABLED: Right (please use Right Glove)"
    fi
}

SummaryHaptics()
{
    echo ""
    echo "===============  HAPTICS CONTROL  ================="
    MaxFingersOptions
    echo "MIN_INTENSITY: $min_intensity Newtons"
    echo "MAX_INTENSITY: $max_intensity Newtons"
    echo "TIME between SENSATIONS: $precision_duration msec"
    echo "INITIAL_DELAY: $intial_delay msec"
    echo "FINAL_DELAY: $final_delay msec"
    echo "INTENSITY_SHIFT: $intensity_shift Newtons"
    IntensityModelOptions
    echo "STATIC_INTENSITY (MODEL 0): $static_intensity Newton"
    #echo "MAX_GRIPPER_WIDTH_VARIATION: $max_width mm" SENT BY LOCAL NODE
    #echo "MAX_GRIPPER_FORCE: $max_force Newtons" SENT BY LOCAL NODE
    HandEnabledOptions
    echo "DISCRETE_SENSATIONS: $discrete_sensations"
    echo "BIDIRECTIONAL_COMMS: $bidirectional_haptic"
    echo "====================================================="
    echo ""
}

############################################################
############################################################
# INITIALIZING REMOTE NODE (Tracking + Haptics Control)    #
############################################################
############################################################

# Set variables
OPTIND=1
device=0
width=960
height=540
initial=4
tolerance=1
tracking=0
color=1
use_static_image_mode="False"
min_detection_confidence=0.7
min_tracking_confidence=0.5
record=""
with_orientation="False"
bidirectional_tracking="False"

max_fingers=5
min_intensity=10.0
max_intensity=100.0
precision_duration=100.0
initial_delay=0.0
final_delay=0.0
intensity_shift=0.0
static_intensity=80.0
intensity_model=0
max_width=100.0
max_force=40.0
right_hand_enabled="True"
discrete_sensations="True"
bidirectional_haptic="False"

############################################################
# Process the input options. Add options as needed.        #
############################################################

while [ $# -gt 0 ] ; do
  case $1 in
    --help) Help ;;
    -d | --device | --cam_device) device="$2" ;;
    -w | --width | --cam_width) width="$2" ;;
    -h | --height | --cam_height) height="$2" ;;
    -i | --initial | --initial_pos) initial="$2" ;;
    -tol | --tolerance) tolerance="$2" ;;
    -t | --track | --tracking) tracking="$2" ;;
    -c | --color | --gloves_color) color="$2" ;;
    -use_static_image_mode) use_static_image_mode="True" ;;
    -min_det | --min_detection_confidence) min_detection_confidence="$2" ;;
    -min_track | --min_tracking_confidence) min_tracking_confidence="$2" ;;
    -r | --record) record="$2" ;;
    --oriented | --with_orientation) with_orientation="$2" ;;
    --bidirect_track | --bidirectional_tracking) bidirectional_tracking="$2" ;;
    --mf | --max_fingers) max_fingers="$2" ;;
    --min_int | --min_intensity) min_intensity="$2" ;;
    --max_int | --max_intensity) max_intensity="$2" ;;
    --prec_dur | --precision_duration) precision_duration="$2" ;;
    --init_delay | initial_delay) initial_delay="$2" ;;
    --fin_delay | --final_delay) final_delay="$2" ;;
    --int_shift | --intensity_shift) intensity_shift="$2" ;;
    --static_int | --static_intensity) static_intensity="$2" ;;
    --int_model | --intensity_model) intensity_model="$2" ;;
    --max_width) max_width="$2" ;;
    --max_force) max_force="$2" ;;
    --right_hand | --right_hand_enabled) right_hand_enabled="$2" ;;
    --discrete | --discrete_sensations) discrete_sensations="$2" ;;
    --bidirect_haptic | --bidirectional_haptic) bidirectional_haptic="$2" ;;
  esac
  shift
done

SummaryHaptics

SummaryTracking

echo "Do you accept these configuration parameters?"
select yn in "Yes" "No"; do
    case $yn in
        Yes ) 
            echo "Initializing Remote Node"; 
            source /home/ros-iteam/catkin_ws/devel/setup.bash
            # CHANGE PYTHONPATH: 
            export PYTHONPATH=/home/ros-iteam/catkin_ws/devel/lib/python3/dist-packages:/opt/ros/noetic/lib/python3/dist-packages:/home/ros-iteam/catkin_ws/src/haptic_ur5e/src
            if [[ "$use_static_image_mode" == "True" ]]; then
                gnome-terminal --tab -- rosrun haptic_ur5e tracking_control_teleop.py "--device" "$device" "--width" "$width" "--height" "$height" \
                "--initial_position" "$initial" "--tolerance" "$tolerance" "--tracking" "$tracking" "--gloves_color" "$color" \
                "--use_static_image_mode" "--min_detection_confidence" "$min_detection_confidence" \
                "--record" "$record" "--with_orientation" "$with_orientation" "--bidirectional" "$bidirectional_tracking"
            else
                gnome-terminal --tab -- rosrun haptic_ur5e tracking_control_teleop.py "--device" "$device" "--width" "$width" "--height" "$height" \
                "--initial_position" "$initial" "--tolerance" "$tolerance" "--tracking" "$tracking" "--gloves_color" "$color" \
                "--min_detection_confidence" "$min_detection_confidence" \
                "--record" "$record" "--with_orientation" "$with_orientation" "--bidirectional" "$bidirectional_tracking"
            fi
            break;;
        No ) echo "Cancelling Operation..."; break;;
    esac
done
