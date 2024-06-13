#!/bin/bash

############################################################
# Help                                                     #
############################################################
Help()
{
    # Display Help
    echo "--mapping <int 0-3>: Mapping algorithm (default: 0, i.e. linear_mapping, focused_linear_mapping, multi_focused_linear_mapping, non_linear_mapping)."
    echo "--controller <int 0-2>: pose-based cartesian controller (0), joint-based cartesian controller (1), forward cartesian controller (2). Default: 0"
    echo "--initial <int 0-8>: up_left, up_center, up_right, ..., low_right (default: 4, i.e. mid_center)."
    echo "--oneaxis <int 0-2>: x, y, z (default: None, i.e. movement in all 3 axis)."
    echo "--twoaxis <int>: xy, yz, xz (default: None, i.e. movement in all 3 axis)."
    echo "--duration <int>: fixed-value (0), dynamic mirroring (1), linear position-based (2). (default: 2)"
    echo "--velocity <int 0-2>: self-adjusting (0), fixed value (1), linear dynamic (2). (default 0)"
    echo "--acceleration <int 0-2>: self-adjusting (0), fixed value (1), linear dynamic (2). (default 0)"
    echo "--precision <float>: Movement tolerance, avoiding trajectories with an euclidean distance less than provided value. (default: 0.001)"
    echo "--wait_time <float>: Waiting time in seconds for the reception of new commands. (default: None, i.e. self-adjusting)"
    echo "--mimic <bool True/False>: Order the robot arm to mimic the user's movements (default: False, i.e. mirror user's movement)."
    echo "--inverted <bool True/False>: Order the robot to invert only the up-down movement (default: False, i.e. no inversion)."
    echo "--bidirectional <bool True/False>: Enable bidirectional communication with arm_tracking.py and haptic_control.py."
    echo ""
}

############################################################
# Configurations                                           #
############################################################

MappingOptions()
{
    if [[ "$mapping" == "1" ]]; then
        echo "MAPPING: Focused Linear Mapping ($mapping)"
    elif [[ "$mapping" == "2" ]]; then
        echo "MAPPING: Multi Focused Linear Mapping ($mapping)"
    elif [[ "$mapping" == "3" ]]; then
        echo "MAPPING: Non-Linear Mapping ($mapping)"
    else
        mapping=0
        echo "MAPPING: Linear Mapping (default: $mapping)"
    fi
}

ControllerOptions()
{
    if [[ "$controller" == "1" ]]; then
        echo "CONTROLLER: Joint-Based Cartesian Controller ($controller)"
    elif [[ "$mapping" == "2" ]]; then
        echo "CONTROLLER: Forward Cartesian Controller ($controller)"
    else
        controller=0
        echo "CONTROLLER: Pose-Based Cartesian Controller (default: $controller)"
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

OneAxisOptions()
{
    if [[ "$oneaxis" == "0" ]]; then
        echo "ONE_AXIS: Robot X Axis ($oneaxis)"
    elif [[ "$oneaxis" == "1" ]]; then
        echo "ONE_AXIS: Robot Y Axis ($oneaxis)"
    elif [[ "$oneaxis" == "2" ]]; then
        echo "ONE_AXIS: Robot Z Axis ($oneaxis)"
    else
        oneaxis=-1
        echo "ONE_AXIS: Disabled (default: $oneaxis)"
    fi
}

TwoAxisOptions()
{
    if [[ "$twoaxis" == "0" ]]; then
        echo "TWO_AXIS: Robot XY Axis ($twoaxis)"
    elif [[ "$twoaxis" == "1" ]]; then
        echo "TWO_AXIS: Robot XZ Axis ($twoaxis)"
    elif [[ "$twoaxis" == "2" ]]; then
        echo "TWO_AXIS: Robot YZ Axis ($twoaxis)"
    else
        twoaxis=-1
        echo "TWO_AXIS: Disabled (default: $twoaxis)"
    fi
}

DurationOptions()
{
    if [[ "$duration" == "0" ]]; then
        echo "DURATION: Fixed Value ($duration)"
    elif [[ "$duration" == "1" ]]; then
        echo "DURATION: Dynamic Mirroring ($duration)"
    else
        duration=2
        echo "DURATION: Linear Position_Based (default: $duration)"
    fi
}

VelocityOptions()
{
    if [[ "$velocity" == "1" ]]; then
        echo "VELOCITY: Fixed Value ($velocity)"
    elif [[ "$velocity" == "2" ]]; then
        echo "VELOCITY: Linear Dynamic ($velocity)"
    else
        velocity=0
        echo "VELOCITY: Self-Adjusting (default: $velocity)"
    fi
}

AccelerationOptions()
{
    if [[ "$acceleration" == "1" ]]; then
        echo "ACCELERATION: Fixed Value ($acceleration)"
    elif [[ "$acceleration" == "2" ]]; then
        echo "ACCELERATION: Linear Dynamic ($acceleration)"
    else
        acceleration=0
        echo "ACCELERATION: Self-Adjusting (default: $acceleration)"
    fi
}

OrientationOptions()
{
    if [[ "$orientation" == "1" ]]; then
        echo "ORIENTATION: Sided Upwards ($orientation)"
    elif [[ "$orientation" == "2" ]]; then
        echo "ORIENTATION: Sided Downwards ($orientation)"
    elif [[ "$orientation" == "3" ]]; then
        echo "ORIENTATION: Horizontal Position ($orientation)"
    else
        orientation=0
        echo "ORIENTATION: Vertical Position (default: $orientation)"
    fi
}

PrecisionOptions()
{
    if [[ "$precision" == "0.001" ]]; then
        echo "PRECISION: Robot Precision (default $precision meters)"
    else
        echo "PRECISION: Custom Value ($precision meters)"
    fi
}


WaitTimeOptions()
{
    if [[ "$wait_time" == "-1" ]]; then
        echo "WAIT_TIME: Self-Adjusting (default $wait_time)"
    else
        echo "WAIT_TIME: Fixed Value ($wait_time seconds)"
    fi
}

MimicOptions()
{
    if [[ "$mimic" == "True" ]]; then
        echo "MIMIC: Enabled (True)"
    else
        mimic="False"
        echo "MIMIC: Disabled (default: False)"
    fi
}


SummaryRobot()
{
    echo ""
    echo "===============  ROBOTIC CONTROL  ================="
    MappingOptions
    ControllerOptions
    InitialPositionOptions
    OneAxisOptions
    TwoAxisOptions
    DurationOptions
    VelocityOptions
    AccelerationOptions
    OrientationOptions
    PrecisionOptions
    WaitTimeOptions
    MimicOptions
    echo "INVERTED_Y_AXIS: $inverted"
    echo "BIDIRECTIONAL_COMMS: $bidirectional_comms"
    echo "====================================================="
    echo ""
}

WidthModelOptions()
{
    if [[ "$width_model" == "1" ]]; then
        echo "WIDTH_MODEL: Dynamic by Interpolating Levels ($width_model)"
    elif [[ "$width_model" == "2" ]]; then
        echo "WIDTH_MODEL: XXX GripWidth-HumanWidth Model ($width_model)"
    elif [[ "$width_model" == "3" ]]; then
        echo "WIDTH_MODEL: XXX GripWidth-HumanWidth Model ($width_model)"
    else
        width_model=0
        echo "WIDTH_MODEL: Static by Grip Levels (default: $width_model)"
    fi
}

ForceModelOptions()
{
    if [[ "$force_model" == "1" ]]; then
        echo "FORCE_MODEL: Dynamic by Execution Time ($force_model)"
    elif [[ "$force_model" == "2" ]]; then
        echo "FORCE_MODEL: XXX Force-TimeVar Model ($force_model)"
    elif [[ "$force_model" == "3" ]]; then
        echo "FORCE_MODEL: XXX Force-TimeVar Model ($force_model)"
    else
        force_model=0
        echo "FORCE_MODEL: Static Force Value (default: $force_model)"
    fi
}

DelayModelOptions()
{
    if [[ "$delay_model" == "1" ]]; then
        echo "DELAY_MODEL: Dynamic by Width Variation ($delay_model)"
    elif [[ "$delay_model" == "2" ]]; then
        echo "DELAY_MODEL: XXX Delay-WidthVar Model ($delay_model)"
    elif [[ "$delay_model" == "3" ]]; then
        echo "DELAY_MODEL: XXX Delay-WidthVar Model ($delay_model)"
    else
        delay_model=1
        echo "DELAY_MODEL: Static at Time_Max_Width (default: $delay_model)"
    fi
}

GripLevelsOptions()
{
    if [[ "$grip_levels" == "1" ]]; then
        echo "GRIP_LEVELS: Only Open/Close Enabled ($grip_levels)"
    elif [[ "$grip_levels" == "2" ]]; then
        echo "GRIP_LEVELS: Open/Open-Medium/Close Enabled ($grip_levels)"
    elif [[ "$grip_levels" == "3" ]]; then
        echo "GRIP_LEVELS: Open/Open-Medium/Medium/Close Enabled ($grip_levels)"
    else
        grip_levels=3
        echo "GRIP_LEVELS: ALL Grip-Levels Enabled (default: $grip_levels)"
    fi
}

SummaryGripper()
{
    # Output the values
    echo ""
    echo "===============  GRIPPER CONTROL  ================="
    echo "MAX_WIDTH: $max_width metres"
    echo "MAX_FORCE: $max_force metres"
    echo "START_WIDTH: $start_width metres"
    echo "WIDTH_TOLERANCE: $width_tolerance metres"
    echo "FORCE_TOLERANCE: $force_tolerance Newton"
    echo "MAX_DURATION: $max_duration milliseconds"
    echo "TIME_MAX_WIDTH: $time_max_width seconds"
    WidthModelOptions
    ForceModelOptions
    DelayModelOptions
    GripLevelsOptions
    echo "====================================================="
    echo ""
}

############################################################
############################################################
# INITIALIZING LOCAL NODE  (Robotic + Gripper Control)     #
############################################################
############################################################

# Set variables
OPTIND=1
mapping=0
controller=0
initial=4
oneaxis=-1
twoaxis=-1
duration=2
velocity=0
acceleration=0
orientation=0
precision=0.001
wait_time=-1
mimic="False"
inverted="False"
bidirectional_comms="False"

max_width=100.0
max_force=40.0
start_width=100.0
width_tolerance=2.0
force_tolerance=2.0
max_duration=60.0
time_max_width=2.0
width_model=0
force_model=0
delay_model=1
grip_levels=3

############################################################
# Process the input options. Add options as needed.        #
############################################################

while [ $# -gt 0 ] ; do
  case $1 in
    --help) Help ;;
    --map | --mapping) mapping="$2" ;;
    -c | --control | --controller) controller="$2" ;;
    -i | --initial | --initial_pos) initial="$2" ;;
    -o | --one | --oneaxis) oneaxis="$2" ;;
    -t | --two | --twoaxis) twoaxis="$2" ;;
    -d | --duration) duration="$2" ;;
    -v | --velocity) velocity="$2" ;;
    -a | --acceleration) acceleration="$2" ;;
    -p | --precision) precision="$2" ;;
    -w | --wait_time) wait_time="$2" ;;
    --angle | --orientation) orientation="$2" ;;
    --mimic) mimic="$2" ;;
    --inverted) inverted="$2" ;;
    --bidirectional_comms) bidirectional_comms="$2" ;;
    --mw | --max_width) max_width="$2" ;;
    --mf | --max_force) max_force="$2" ;;
    --sw | --start_width) start_width="$2" ;;
    --wt | --width_tol | --width_tolerance) width_tolerance="$2" ;;
    --ft | --force_tol | --force_tolerance) force_tolerance="$2" ;;
    --md | --max_dur | --max_duration) max_duration="$2" ;;
    --tmw | --time_max_width) time_max_width="$2" ;;
    --wm | --width_model) width_model="$2" ;;
    --fm | --force_model) force_model="$2" ;;
    --dm | --delay_model) delay_model="$2" ;;
    --gl | --grip_levels) grip_levels="$2" ;;
  esac
  shift
done

SummaryRobot

SummaryGripper

echo "Do you accept these configuration parameters?"
select yn in "Yes" "No"; do
    case $yn in
        Yes ) 
            echo "Initializing Local Node"; 
            source /home/ros-iteam/catkin_ws/devel/setup.bash
            export PYTHONPATH=/home/ros-iteam/catkin_ws/devel/lib/python3/dist-packages:/opt/ros/noetic/lib/python3/dist-packages:/home/ros-iteam/catkin_ws/src/haptic_ur5e/src
            gnome-terminal --tab -- rosrun haptic_ur5e robot_control_teleop.py "--mapping" "$mapping" "--controller" "$controller" \
            "--initial_pos" "$initial" "--oneaxis" "$oneaxis" "--twoaxis" "$twoaxis" "--duration" "$duration" "--velocity" "$velocity" \
            "--acceleration" "$acceleration" "--orientation" "$orientation" "--mimic" "$mimic" "--inverted" "$inverted" "--bidirectional" "$bidirectional_comms"
            sleep 2
            gnome-terminal --tab -- rosrun haptic_ur5e gripper_control_teleop.py "--max_width" $max_width "--max_force" $max_force "--start_width" $start_width \
            "--width_tolerance" $width_tolerance "--force_tolerance" $force_tolerance "--max_duration" $max_duration "--time_max_width" $time_max_width \
            "--width_model" $width_model "--force_model" $force_model "--delay_model" $delay_model "--grip_levels" $grip_levels "--bidirectional" "$bidirectional_comms"
            break;;
        No ) echo "Cancelling Operation..."; break;;
    esac
done
