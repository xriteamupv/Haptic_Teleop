#!/bin/bash

############################################################
# Help                                                     #
############################################################
Help()
{
    # Display Help
    echo "--robot_model: Model of Robotic Arm: e.g. ur3e, ur5e, ur10e (Default: ur5e)"
    echo "--robot_ip: IPv4 Address of Robotic Arm (default: 192.168.56.10)"
    echo "--kinematics_config: Directory of Calibration Configuration File (default: /home/ros-iteam/catkin_ws/yaml/robot_calibration.yaml)."
    echo ""
}

ModelOptions()
{
    if [[ "$robot_model" == "ur3e" ]]; then
        echo "ROBOT_MODEL: Universal Robots UR3e ($robot_model)"
    elif [[ "$robot_model" == "ur10e" ]]; then
        echo "ROBOT_MODEL: Universal Robots UR10e ($robot_model)"
    else
        robot_model="ur5e"
        echo "ROBOT_MODEL: Universal Robots UR5e (default: $robot_model)"
    fi
}

AddressOptions()
{
    if [[ "$robot_ip" == "" ]]; then
        robot_ip="192.168.56.10"
        echo "ROBOT_IP: $robot_ip (default)"
    else
        echo "ROBOT_IP: $robot_ip"
    fi
}

ConfigOptions()
{
    if [[ "$kinematics_config" == "" ]]; then
        kinematics_config="/home/ros-iteam/catkin_ws/yaml/robot_calibration.yaml"
        echo "CONFIG: $kinematics_config (default)"
    else
        echo "CONFIG: $kinematics_config"
    fi
}

SummaryControllers()
{
    echo ""
    echo "==========  CONTROLLERS CONFIGURATION  ============"
    ModelOptions
    AddressOptions
    ConfigOptions
    echo "====================================================="
    echo ""
}

# Set variables
OPTIND=1
robot_model="ur5e"
robot_ip="192.168.56.10"
kinematics_config="/home/ros-iteam/catkin_ws/yaml/robot_calibration.yaml"

while [ $# -gt 0 ] ; do
  case $1 in
    --help) Help ;;
    --m | --model | --robot_model) robot_model="$2" ;;
    -ip | --robot_ip | --controller) robot_ip="$2" ;;
    -k | --calibration | --kinematics_config) kinematics_config="$2" ;;
  esac
  shift
done

SummaryControllers

echo "Do you accept these configuration parameters?"
select yn in "Yes" "No"; do
    case $yn in
        Yes ) 
            echo "Initializing Controllers"; 
            source /home/ros-iteam/catkin_ws/devel/setup.bash
            gnome-terminal --tab -- roslaunch ur_robot_driver ${robot_model}_bringup.launch robot_ip:=$robot_ip kinematics_config:=$kinematics_config
            sleep 5
            gnome-terminal --tab -- roslaunch xmlrpc_server xmlrpc_server.launch
            sleep 3
            gnome-terminal --tab -- roslaunch ur_gripper_controller xmlrpc_controller.launch
            break;;
        No ) echo "Cancelling Operation..."; break;;
    esac
done



