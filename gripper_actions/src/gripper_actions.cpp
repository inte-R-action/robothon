/*
 *********************************************************************************
 * Author: inte-R-action 
 * Date: 13-May-2022
 *
 * University of Bath
 * Multimodal Interaction and Robotic Active Perception (inte-R-action) Lab
 * Centre for Autonomous Robotics (CENTAUR)
 * Department of Electronics and Electrical Engineering
 *
 * Description: Control of RobotiQ 2F gripper responding to contact detection
 *
 *********************************************************************************
 */

#include "ros/ros.h"
#include <sstream>
#include <fstream>
#include <iostream>
#include <string.h>
#include <unistd.h>
#include "std_msgs/String.h"
#include "std_msgs/Int32.h"
#include "std_msgs/Float32.h"
#include "std_msgs/Bool.h"
#include "robotiq_2f_gripper_control/Robotiq2FGripper_robot_output.h"
#include "robotiq_2f_gripper_control/Robotiq2FGripper_robot_input.h"
#include "robotiq_ft_sensor/ft_sensor.h"
#include "robotiq_ft_sensor/sensor_accessor.h"

#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>
#include <moveit_msgs/DisplayRobotState.h>
#include <moveit_msgs/DisplayTrajectory.h>
#include <moveit_msgs/AttachedCollisionObject.h>
#include <moveit_msgs/CollisionObject.h>
#include <moveit_visual_tools/moveit_visual_tools.h>
#include <moveit/trajectory_processing/iterative_time_parameterization.h>
#include <moveit_msgs/ExecuteTrajectoryActionResult.h>

#include "gripper_actions/gripperControlParameters.h"

using namespace std;

gripper_actions::gripperControlParameters gripper_action_mode;

// global variable to hold the status of the gripper
robotiq_2f_gripper_control::Robotiq2FGripper_robot_input gripper_status;
robotiq_2f_gripper_control::Robotiq2FGripper_robot_output outputControlValues;

// callback function to get the status signals from the gripper
void gripperStatusCallback(const robotiq_2f_gripper_control::Robotiq2FGripper_robot_input::ConstPtr& msg)
{
    gripper_status = *msg;
}

// callback function to control the gripper action
void gripperControlParametersCallback(const gripper_actions::gripperControlParameters::ConstPtr& msg)
{
    gripper_action_mode.action = msg->action;
    gripper_action_mode.position = msg->position;
    gripper_action_mode.speed = msg->speed;
    gripper_action_mode.force = msg->force;
    gripper_action_mode.useContactDetection = msg->useContactDetection;

    cout << "Gripper action: " << gripper_action_mode.action << endl;
}



int main(int argc, char** argv)
{
    ros::init(argc, argv, "gripper_actions");

    ros::NodeHandle node_handle;

    ros::Rate loop_rate(10);
    ros::AsyncSpinner spinner(1);
    spinner.start();

    std_msgs::Bool msgGripperActionStatus;

    // connection of publisher and subscriber with the Robotiq controller from ROS Industrial
    ros::Publisher Robotiq2FGripperArgPub = node_handle.advertise<robotiq_2f_gripper_control::Robotiq2FGripper_robot_output>("Robotiq2FGripperRobotOutput", 1);
    ros::Subscriber Robotiq2FGripperStatusSub = node_handle.subscribe("Robotiq2FGripperRobotInput", 1, gripperStatusCallback);
    ros::Subscriber gripperControlParametersSub = node_handle.subscribe<gripper_actions::gripperControlParameters>("gripper_actions", 1, gripperControlParametersCallback);
    ros::Publisher gripperActionStatusPub = node_handle.advertise<std_msgs::Bool>("gripper_actions_status", 1000);

    while( ros::ok() )
    {
        if( gripper_action_mode.action == "reset" )    // reset the robotic gripper (needed to activate the robot)
        {
            std::cout << "RESET GRIPPER" << std::endl;
            outputControlValues.rACT = 0;
            outputControlValues.rGTO = 0;
            outputControlValues.rATR = 0;
            outputControlValues.rPR = 0;
            outputControlValues.rSP = 0;
            outputControlValues.rFR = 0;
            Robotiq2FGripperArgPub.publish(outputControlValues);
            std::cout << "RESET GRIPPER" << std::endl;
            // give some time the gripper to reset
            sleep(3);

            printf("COMPLETED: gSTA [%d]\n", gripper_status.gSTA);
            msgGripperActionStatus.data = true;
            gripperActionStatusPub.publish(msgGripperActionStatus);
            sleep(0.5);

            gripper_action_mode.action = "empty";
            gripper_action_mode.position = 0;
            gripper_action_mode.speed = 0;
            gripper_action_mode.force = 0;
            gripper_action_mode.useContactDetection = 0;
        }
        else if( gripper_action_mode.action == "activate" )    // activate the robotic gripper
        {
            std::cout << "ACTIVATE GRIPPER" << std::endl; 
            outputControlValues.rACT = 1;
            outputControlValues.rGTO = 1;
            outputControlValues.rATR = 0;
            outputControlValues.rPR = 0;
            outputControlValues.rSP = 200;
            outputControlValues.rFR = 50;
            Robotiq2FGripperArgPub.publish(outputControlValues);
            std::cout << "ACTIVATE GRIPPER" << std::endl; 

            // wait until the activation action is completed to continue with the next action
            while( gripper_status.gSTA != 3 )
            {
            }

            printf("COMPLETED: gSTA [%d]\n", gripper_status.gSTA);
            msgGripperActionStatus.data = true;
            gripperActionStatusPub.publish(msgGripperActionStatus);
            sleep(0.5);

            gripper_action_mode.action = "empty";
            gripper_action_mode.position = 0;
            gripper_action_mode.speed = 0;
            gripper_action_mode.force = 0;
            gripper_action_mode.useContactDetection = 0;
        }
        else if( gripper_action_mode.action == "close" )    // activate the robotic gripper
        {
            std::cout << "CLOSE GRIPPER" << std::endl; 
            outputControlValues.rGTO = 1;
            outputControlValues.rSP = gripper_action_mode.speed;
            outputControlValues.rPR = gripper_action_mode.position;
            outputControlValues.rFR = gripper_action_mode.force;
            Robotiq2FGripperArgPub.publish(outputControlValues);
            std::cout << "CLOSE GRIPPER" << std::endl; 

            // wait until the activation action is completed to continue with the next action
            if( gripper_action_mode.useContactDetection == true )
            {
                while( gripper_status.gOBJ != 3 && gripper_status.gOBJ != 2 )
                {
                }
            }
            else
            {
                while( gripper_status.gOBJ != 3 )
                {
                }                
            }                

            printf("COMPLETED: gSTA [%d]\n", gripper_status.gSTA);
            msgGripperActionStatus.data = true;
            gripperActionStatusPub.publish(msgGripperActionStatus);
            sleep(0.5);

            gripper_action_mode.action = "empty";
            gripper_action_mode.position = 0;
            gripper_action_mode.speed = 0;
            gripper_action_mode.force = 0;
            gripper_action_mode.useContactDetection = 0;
        }
        else if( gripper_action_mode.action == "open" )    // activate the robotic gripper
        {
            std::cout << "OPEN GRIPPER" << std::endl; 
            outputControlValues.rGTO = 1;
            outputControlValues.rSP = gripper_action_mode.speed;
            outputControlValues.rPR = gripper_action_mode.position;
            outputControlValues.rFR = gripper_action_mode.force;
            Robotiq2FGripperArgPub.publish(outputControlValues);
            std::cout << "OPEN GRIPPER" << std::endl; 

            // wait until the activation action is completed to continue with the next action
            if( gripper_action_mode.useContactDetection == true )
            {
                while( gripper_status.gOBJ != 3 && gripper_status.gOBJ != 2 )
                {
                }
            }
            else
            {
                while( gripper_status.gOBJ != 3 )
                {
                }                
            }

            printf("COMPLETED: gSTA [%d]\n", gripper_status.gSTA);
            msgGripperActionStatus.data = true;
            gripperActionStatusPub.publish(msgGripperActionStatus);
            sleep(0.5);

            gripper_action_mode.action = "empty";
            gripper_action_mode.position = 0;
            gripper_action_mode.speed = 0;
            gripper_action_mode.force = 0;
            gripper_action_mode.useContactDetection = 0;
        }
        else
        {

        }

        // set gripper to standby to clear the flags
        outputControlValues.rGTO = 0;
        Robotiq2FGripperArgPub.publish(outputControlValues);
        std::cout << "STANDBY GRIPPER" << std::endl; 
        sleep(0.5);
    }

    usleep(1000);
    ros::waitForShutdown();

    return 0;
}
