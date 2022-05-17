/*
 *********************************************************************************
 * Author: Uriel Martinez-Hernandez
 * Email: u.martinez@bath.ac.uk
 * Date: 13-May-2022
 *
 * University of Bath
 * Multimodal Interaction and Robotic Active Perception (inte-R-action) Lab
 * Centre for Autonomous Robotics (CENTAUR)
 * Department of Electronics and Electrical Engineering
 *
 * Description: Example of RobotiQ 2F gripper responding to contact detection from tactile sensor
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

#include "ur_msgs/SetSpeedSliderFraction.h" //type of the service for speed slider

#include <pluginlib/class_list_macros.hpp>
#include <ur_client_library/ur/tool_communication.h>
#include <ur_client_library/exceptions.h>

#include "robot_actions/robotControlParameters.h"
#include "gripper_actions/gripperControlParameters.h"


using namespace std;

bool gripperActionsReady = false;
bool robotActionsReady = false;


/*
// callback function to control the gripper action
void robotControlParametersCallback(const robot_tasks::robotControlParameters::ConstPtr& msg)
{
    robot_action_mode.action = msg->action;
    robot_action_mode.position = msg->position;
    robot_action_mode.speed = msg->speed;
    robot_action_mode.forceDetection = msg->forceDetection;
    robot_action_mode.incrementZaxis = msg->incrementZaxis;
}
*/

// callback function from gripper actions status
void gripperActionsStatusCallback(const std_msgs::Bool::ConstPtr& msg)
{
	gripperActionsReady = msg->data;
}

// callback function from gripper actions status
void robotActionsStatusCallback(const std_msgs::Bool::ConstPtr& msg)
{
	robotActionsReady = msg->data;
}

int main(int argc, char** argv)
{
    ros::init(argc, argv, "robot_tasks");

    ros::NodeHandle node_handle;

    ros::Rate loop_rate(10);
    ros::AsyncSpinner spinner(0);
    spinner.start();

	gripper_actions::gripperControlParameters gripper_actions_msg;
	robot_actions::robotControlParameters robot_actions_msg;

    ros::Publisher gripper_actions_pub = node_handle.advertise<gripper_actions::gripperControlParameters>("gripper_actions", 1000);
    ros::Publisher robot_actions_pub = node_handle.advertise<robot_actions::robotControlParameters>("robot_actions", 10);

    ros::Subscriber gripper_actions_status_sub = node_handle.subscribe("gripper_actions_status", 1, gripperActionsStatusCallback);
    ros::Subscriber robot_actions_status_sub = node_handle.subscribe("robot_actions_status", 1, robotActionsStatusCallback);


    // connection of publisher and subscriber with the Robotiq controller from ROS Industrial
//    ros::Subscriber robotControlParametersSub = node_handle.subscribe<robot_tasks::robotControlParameters>("robot_tasks", 1, robotControlParametersCallback);
//    ros::Publisher robotActionStatusPub = node_handle.advertise<std_msgs::Bool>("robotActionStatus", 1000);

    robotiq_ft_sensor::sensor_accessor srv;
    ros::ServiceClient clientSpeedSlider  = node_handle.serviceClient<ur_msgs::SetSpeedSliderFraction>("/ur_hardware_interface/set_speed_slider"); //robot velocity control

    // UR3 robot set up
    static const std::string PLANNING_GROUP = "manipulator";
    moveit::planning_interface::MoveGroupInterface move_group(PLANNING_GROUP);
    moveit::planning_interface::PlanningSceneInterface planning_scene_interface;
    const robot_state::JointModelGroup* joint_model_group = move_group.getCurrentState()->getJointModelGroup(PLANNING_GROUP);

    namespace rvt = rviz_visual_tools;
    moveit_visual_tools::MoveItVisualTools visual_tools("world");
    visual_tools.deleteAllMarkers();

    visual_tools.loadRemoteControl();
    Eigen::Isometry3d text_pose = Eigen::Isometry3d::Identity();
    text_pose.translation().z() = 1.75;
    visual_tools.publishText(text_pose, "inte-R-action - v 0.1.0", rvt::WHITE, rvt::XLARGE);
    visual_tools.trigger();

    ROS_INFO_NAMED("UR3 robot", "Reference frame: %s", move_group.getPlanningFrame().c_str());
    ROS_INFO_NAMED("UR3 robot", "End effector link: %s", move_group.getEndEffectorLink().c_str());
    ROS_INFO_NAMED("UR3 robot", "Available Planning Groups:");
    std::copy(move_group.getJointModelGroupNames().begin(), move_group.getJointModelGroupNames().end(), std::ostream_iterator<std::string>(std::cout, ", "));

    moveit::planning_interface::MoveGroupInterface::Plan plan;
    moveit::core::RobotStatePtr current_state = move_group.getCurrentState();


    std::vector<double> joint_group_positions;
    current_state->copyJointGroupPositions(joint_model_group, joint_group_positions);

    move_group.setMaxVelocityScalingFactor(0.50);
    move_group.setMaxAccelerationScalingFactor(0.50);

    ur_msgs::SetSpeedSliderFraction set_speed_frac;
    set_speed_frac.request.speed_slider_fraction = 0.20; // change velocity in the slider of teach pendant
    clientSpeedSlider.call(set_speed_frac);   


	// reset the gripper using default values
	cout << "gripper action: empty" << endl;
	gripper_actions_msg.action = "empty";
	gripper_actions_msg.position = 0;
	gripper_actions_msg.speed = 0;
	gripper_actions_msg.force = 0;
	gripper_actions_msg.useContactDetection = 0;
	cout << "gripper action: reset2" << endl;
	gripper_actions_pub.publish(gripper_actions_msg);
	sleep(1);

    while( ros::ok() )
    {
		gripperActionsReady = false;
		cout << "Running robot tasks module" << endl;

		// reset the gripper using default values
		cout << "gripper action: reset" << endl;
		gripper_actions_msg.action = "reset";
		gripper_actions_msg.position = 0;
		gripper_actions_msg.speed = 0;
		gripper_actions_msg.force = 0;
		gripper_actions_msg.useContactDetection = 0;
		gripper_actions_pub.publish(gripper_actions_msg);
		while( !gripperActionsReady )
		{
			cout << "gripperActionReady = " << gripperActionsReady << endl;
		}
		gripperActionsReady = false;

		// activate the gripper using default values
		cout << "gripper action: activate" << endl;
		gripper_actions_msg.action = "activate";
		gripper_actions_msg.position = 0;
		gripper_actions_msg.speed = 0;
		gripper_actions_msg.force = 0;
		gripper_actions_msg.useContactDetection = 0;
		gripper_actions_pub.publish(gripper_actions_msg);
		while( !gripperActionsReady )
		{
			cout << "gripperActionReady = " << gripperActionsReady << endl;
		}	
		gripperActionsReady = false;

		// open the gripper without force detection
		cout << "gripper action: open" << endl;
		gripper_actions_msg.action = "open";
		gripper_actions_msg.position = 100;
		gripper_actions_msg.speed = 200;
		gripper_actions_msg.force = 150;
		gripper_actions_msg.useContactDetection = 0;
		gripper_actions_pub.publish(gripper_actions_msg);
		while( !gripperActionsReady )
		{
			cout << "gripperActionReady = " << gripperActionsReady << endl;
		}	
		gripperActionsReady = false;
		sleep(1);
    
		// close the gripper with force detection
		cout << "gripper action: close" << endl;
		gripper_actions_msg.action = "close";
		gripper_actions_msg.position = 240;
		gripper_actions_msg.speed = 200;
		gripper_actions_msg.force = 150;
		gripper_actions_msg.useContactDetection = 1;
		gripper_actions_pub.publish(gripper_actions_msg);
		while( !gripperActionsReady )
		{
			cout << "gripperActionReady = " << gripperActionsReady << endl;
		}	
		gripperActionsReady = false;
		sleep(1);

        /****************************************************/
        // Begining of task 1: box localisation and press blue button

		// send robot to home position
		cout << "robot action: moveToHomePosition" << endl;
        sleep(0.5);
		robot_actions_msg.action = "moveToHomePosition";
		robot_actions_pub.publish(robot_actions_msg);
    	while( !robotActionsReady )
		{
			cout << "robotActionReady = " << robotActionsReady << endl;
		}	
		robotActionsReady = false;

        // move robot in x and y position on the blue button
		cout << "robot action: move to x and y positions" << endl;
        sleep(0.5);
		robot_actions_msg.action = "moveToCartesian";
		robot_actions_msg.position = 0;
		robot_actions_msg.speed = 0;
		robot_actions_msg.force = 0;
		robot_actions_msg.forceDetection = false;
		robot_actions_msg.incrementXaxis = 0.020;
		robot_actions_msg.incrementYaxis = 0.020;
		robot_actions_msg.incrementZaxis = 0.0;
		robot_actions_pub.publish(robot_actions_msg);
		while( !robotActionsReady )
		{
			cout << "robotActionReady = " << robotActionsReady << endl;
		}	
		robotActionsReady = false;

        // move robot down on z axis
		cout << "robot action: move down on z axis" << endl;
        sleep(0.5);
		robot_actions_msg.action = "moveToCartesian";
		robot_actions_msg.position = 0;
		robot_actions_msg.speed = 0;
		robot_actions_msg.force = 0;
		robot_actions_msg.forceDetection = true;
		robot_actions_msg.incrementXaxis = 0.0;
		robot_actions_msg.incrementYaxis = 0.0;
		robot_actions_msg.incrementZaxis = -0.078;
		robot_actions_pub.publish(robot_actions_msg);
		while( !robotActionsReady )
		{
			cout << "robotActionReady = " << robotActionsReady << endl;
		}	
		robotActionsReady = false;

		// send robot to home position
		cout << "robot action: moveToHomePosition" << endl;
        sleep(0.5);
		robot_actions_msg.action = "moveToHomePosition";
		robot_actions_pub.publish(robot_actions_msg);
		while( !robotActionsReady )
		{
			cout << "robotActionReady = " << robotActionsReady << endl;
		}	
		robotActionsReady = false;

        // End of task 1: localisation and press blue button
        /****************************************************/
 
        /****************************************************/
        // Begining of task 4: remove battery case and extract batteries

		// send robot to home position
		cout << "robot action: moveToHomePosition" << endl;
        sleep(0.5);
		robot_actions_msg.action = "moveToHomePosition";
		robot_actions_pub.publish(robot_actions_msg);
		while( !robotActionsReady )
		{
			cout << "robotActionReady = " << robotActionsReady << endl;
		}	
		robotActionsReady = false;

       // move robot in x and y position on the blue button
		cout << "robot action: move to x and y positions on lid" << endl;
        sleep(0.5);
		robot_actions_msg.action = "moveToCartesian";
		robot_actions_msg.position = 0;
		robot_actions_msg.speed = 0;
		robot_actions_msg.force = 0;
		robot_actions_msg.forceDetection = false;
		robot_actions_msg.incrementXaxis = 0.035;
		robot_actions_msg.incrementYaxis = -0.020;
		robot_actions_msg.incrementZaxis = 0.0;
		robot_actions_pub.publish(robot_actions_msg);
		while( !robotActionsReady )
		{
			cout << "robotActionReady = " << robotActionsReady << endl;
		}	
		robotActionsReady = false;

       // move robot in x and y position on the blue button
		cout << "robot action: move down, detect force" << endl;
        sleep(0.5);
		robot_actions_msg.action = "moveToCartesian";
		robot_actions_msg.position = 0;
		robot_actions_msg.speed = 0;
		robot_actions_msg.force = 0;
		robot_actions_msg.forceDetection = true;
		robot_actions_msg.incrementXaxis = 0.0;
		robot_actions_msg.incrementYaxis = 0.0;
		robot_actions_msg.incrementZaxis = -0.078;
		robot_actions_pub.publish(robot_actions_msg);
		while( !robotActionsReady )
		{
			cout << "robotActionReady = " << robotActionsReady << endl;
		}	
		robotActionsReady = false;

       // move robot in x and y position on the blue button
		cout << "robot action: move down, no force detection" << endl;
        sleep(0.5);
		robot_actions_msg.action = "moveToCartesian";
		robot_actions_msg.position = 0;
		robot_actions_msg.speed = 0;
		robot_actions_msg.force = 0;
		robot_actions_msg.forceDetection = false;
		robot_actions_msg.incrementXaxis = 0.0;
		robot_actions_msg.incrementYaxis = 0.0;
		robot_actions_msg.incrementZaxis = -0.005;
		robot_actions_pub.publish(robot_actions_msg);
		while( !robotActionsReady )
		{
			cout << "robotActionReady = " << robotActionsReady << endl;
		}	
		robotActionsReady = false;

       // move robot in x and y position on the blue button
		cout << "robot action: slide on lid" << endl;
        sleep(0.5);
		robot_actions_msg.action = "moveToCartesian";
		robot_actions_msg.position = 0;
		robot_actions_msg.speed = 0;
		robot_actions_msg.force = 0;
		robot_actions_msg.forceDetection = false;
		robot_actions_msg.incrementXaxis = 0.0;
		robot_actions_msg.incrementYaxis = -0.025;
		robot_actions_msg.incrementZaxis = 0.0;
		robot_actions_pub.publish(robot_actions_msg);
		while( !robotActionsReady )
		{
			cout << "robotActionReady = " << robotActionsReady << endl;
		}	
		robotActionsReady = false;

       // move robot in x and y position on the blue button
		cout << "robot action: move robot up" << endl;
        sleep(0.5);
		robot_actions_msg.action = "moveToCartesian";
		robot_actions_msg.position = 0;
		robot_actions_msg.speed = 0;
		robot_actions_msg.force = 0;
		robot_actions_msg.forceDetection = false;
		robot_actions_msg.incrementXaxis = 0.0;
		robot_actions_msg.incrementYaxis = 0.0;
		robot_actions_msg.incrementZaxis = 0.030;
		robot_actions_pub.publish(robot_actions_msg);
		while( !robotActionsReady )
		{
			cout << "robotActionReady = " << robotActionsReady << endl;
		}	
		robotActionsReady = false;
 
      // move robot in x and y position on the blue button
		cout << "robot action: rotate end effector" << endl;
        sleep(0.5);
		robot_actions_msg.action = "moveToJoints";
		robot_actions_msg.position = 0;
		robot_actions_msg.speed = 0;
		robot_actions_msg.force = 0;
		robot_actions_msg.forceDetection = false;
		robot_actions_msg.incrementXaxis = 0.0;
		robot_actions_msg.incrementYaxis = 0.0;
		robot_actions_msg.incrementZaxis = 0.0;
		robot_actions_msg.robotJoints[0] = 0.0;
		robot_actions_msg.robotJoints[1] = 0.0;
		robot_actions_msg.robotJoints[2] = 0.0;
		robot_actions_msg.robotJoints[3] = 0.0;
		robot_actions_msg.robotJoints[4] = 0.0;
		robot_actions_msg.robotJoints[5] = -90.0;
		robot_actions_pub.publish(robot_actions_msg);
		while( !robotActionsReady )
		{
			cout << "robotActionReady = " << robotActionsReady << endl;
		}	
		robotActionsReady = false;
 
		// open the gripper without force detection
		cout << "gripper action: open" << endl;
		gripper_actions_msg.action = "open";
		gripper_actions_msg.position = 150;
		gripper_actions_msg.speed = 200;
		gripper_actions_msg.force = 150;
		gripper_actions_msg.useContactDetection = true;
		gripper_actions_pub.publish(gripper_actions_msg);
		while( !gripperActionsReady )
		{
			cout << "gripperActionReady = " << gripperActionsReady << endl;
		}	
		gripperActionsReady = false;
		sleep(1);

       // move robot in x and y position on the blue button
		cout << "robot action: move down, detect force" << endl;
        sleep(0.5);
		robot_actions_msg.action = "moveToCartesian";
		robot_actions_msg.position = 0;
		robot_actions_msg.speed = 0;
		robot_actions_msg.force = 0;
		robot_actions_msg.forceDetection = true;
		robot_actions_msg.incrementXaxis = 0.0;
		robot_actions_msg.incrementYaxis = 0.0;
		robot_actions_msg.incrementZaxis = -0.045;
		robot_actions_pub.publish(robot_actions_msg);
		while( !robotActionsReady )
		{
			cout << "robotActionReady = " << robotActionsReady << endl;
		}	
		robotActionsReady = false;

		// open the gripper without force detection
		cout << "gripper action: close" << endl;
		gripper_actions_msg.action = "close";
		gripper_actions_msg.position = 220;
		gripper_actions_msg.speed = 50;
		gripper_actions_msg.force = 100;
		gripper_actions_msg.useContactDetection = true;
		gripper_actions_pub.publish(gripper_actions_msg);
		while( !gripperActionsReady )
		{
			cout << "gripperActionReady = " << gripperActionsReady << endl;
		}	
		gripperActionsReady = false;
		sleep(1);

       // move robot in x and y position on the blue button
		cout << "robot action: move up" << endl;
        sleep(0.5);
		robot_actions_msg.action = "moveToCartesian";
		robot_actions_msg.position = 0;
		robot_actions_msg.speed = 0;
		robot_actions_msg.force = 0;
		robot_actions_msg.forceDetection = false;
		robot_actions_msg.incrementXaxis = 0.0;
		robot_actions_msg.incrementYaxis = 0.0;
		robot_actions_msg.incrementZaxis = 0.040;
		robot_actions_pub.publish(robot_actions_msg);
		while( !robotActionsReady )
		{
			cout << "robotActionReady = " << robotActionsReady << endl;
		}	
		robotActionsReady = false;

		// send robot to home position
		cout << "robot action: moveToHomePosition" << endl;
        sleep(0.5);
		robot_actions_msg.action = "moveToHomePosition";
		robot_actions_pub.publish(robot_actions_msg);
		while( !robotActionsReady )
		{
			cout << "robotActionReady = " << robotActionsReady << endl;
		}	
		robotActionsReady = false;

       // move robot in x and y position on the blue button
		cout << "robot action: move down to table" << endl;
        sleep(0.5);
		robot_actions_msg.action = "moveToCartesian";
		robot_actions_msg.position = 0;
		robot_actions_msg.speed = 0;
		robot_actions_msg.force = 0;
		robot_actions_msg.forceDetection = true;
		robot_actions_msg.incrementXaxis = 0.0;
		robot_actions_msg.incrementYaxis = 0.0;
		robot_actions_msg.incrementZaxis = -0.090;
		robot_actions_pub.publish(robot_actions_msg);
		while( !robotActionsReady )
		{
			cout << "robotActionReady = " << robotActionsReady << endl;
		}	
		robotActionsReady = false;

		// open the gripper without force detection
		cout << "gripper action: open" << endl;
		gripper_actions_msg.action = "open";
		gripper_actions_msg.position = 100;
		gripper_actions_msg.speed = 200;
		gripper_actions_msg.force = 150;
		gripper_actions_msg.useContactDetection = true;
		gripper_actions_pub.publish(gripper_actions_msg);
		while( !gripperActionsReady )
		{
			cout << "gripperActionReady = " << gripperActionsReady << endl;
		}	
		gripperActionsReady = false;
		sleep(1);

		// send robot to home position
		cout << "robot action: moveToHomePosition" << endl;
        sleep(0.5);
		robot_actions_msg.action = "moveToHomePosition";
		robot_actions_pub.publish(robot_actions_msg);
		while( !robotActionsReady )
		{
			cout << "robotActionReady = " << robotActionsReady << endl;
		}	
		robotActionsReady = false;

      // move robot in x and y position on the blue button
/*		cout << "robot action: rotate end effector to original orientation" << endl;
        sleep(0.5);
		robot_actions_msg.action = "moveToJoints";
		robot_actions_msg.position = 0;
		robot_actions_msg.speed = 0;
		robot_actions_msg.force = 0;
		robot_actions_msg.forceDetection = false;
		robot_actions_msg.incrementXaxis = 0.0;
		robot_actions_msg.incrementYaxis = 0.0;
		robot_actions_msg.incrementZaxis = 0.0;
		robot_actions_msg.robotJoints[0] = 0.0;
		robot_actions_msg.robotJoints[1] = 0.0;
		robot_actions_msg.robotJoints[2] = 0.0;
		robot_actions_msg.robotJoints[3] = 0.0;
		robot_actions_msg.robotJoints[4] = 0.0;
		robot_actions_msg.robotJoints[5] = 90.0;
		robot_actions_pub.publish(robot_actions_msg);
		while( !robotActionsReady )
		{
			cout << "robotActionReady = " << robotActionsReady << endl;
		}	
		robotActionsReady = false;
*/
		// close gripper to sepcific distance to extract batteries
		cout << "gripper action: open to distance to extract battery" << endl;
		gripper_actions_msg.action = "open";
		gripper_actions_msg.position = 180;
		gripper_actions_msg.speed = 200;
		gripper_actions_msg.force = 150;
		gripper_actions_msg.useContactDetection = false;
		gripper_actions_pub.publish(gripper_actions_msg);
		while( !gripperActionsReady )
		{
			cout << "gripperActionReady = " << gripperActionsReady << endl;
		}	
		gripperActionsReady = false;
		sleep(1);

       // move robot in x and y position on the blue button
		cout << "robot action: move to position to extract battery" << endl;
        sleep(0.5);
		robot_actions_msg.action = "moveToCartesian";
		robot_actions_msg.position = 0;
		robot_actions_msg.speed = 0;
		robot_actions_msg.force = 0;
		robot_actions_msg.forceDetection = false;
		robot_actions_msg.incrementXaxis = 0.030;
		robot_actions_msg.incrementYaxis = -0.020;
		robot_actions_msg.incrementZaxis = 0.0;
		robot_actions_pub.publish(robot_actions_msg);
		while( !robotActionsReady )
		{
			cout << "robotActionReady = " << robotActionsReady << endl;
		}	
		robotActionsReady = false;

       // move robot in x and y position on the blue button
		cout << "robot action: move down to detect battery position" << endl;
        sleep(0.5);
		robot_actions_msg.action = "moveToCartesian";
		robot_actions_msg.position = 0;
		robot_actions_msg.speed = 0;
		robot_actions_msg.force = 0;
		robot_actions_msg.forceDetection = true;
		robot_actions_msg.incrementXaxis = 0.0;
		robot_actions_msg.incrementYaxis = 0.0;
		robot_actions_msg.incrementZaxis = -0.078;
		robot_actions_pub.publish(robot_actions_msg);
		while( !robotActionsReady )
		{
			cout << "robotActionReady = " << robotActionsReady << endl;
		}	
		robotActionsReady = false;

       // move robot in x and y position on the blue button
		cout << "robot action: move down to detect battery position" << endl;
        sleep(0.5);
		robot_actions_msg.action = "moveToCartesian";
		robot_actions_msg.position = 0;
		robot_actions_msg.speed = 0;
		robot_actions_msg.force = 0;
		robot_actions_msg.forceDetection = false;
		robot_actions_msg.incrementXaxis = 0.0;
		robot_actions_msg.incrementYaxis = 0.0;
		robot_actions_msg.incrementZaxis = -0.010;
		robot_actions_pub.publish(robot_actions_msg);
		while( !robotActionsReady )
		{
			cout << "robotActionReady = " << robotActionsReady << endl;
		}	
		robotActionsReady = false;

       // move robot in x and y position on the blue button
		cout << "robot action: slide to press battery side" << endl;
        sleep(0.5);
		robot_actions_msg.action = "moveToCartesian";
		robot_actions_msg.position = 0;
		robot_actions_msg.speed = 0;
		robot_actions_msg.force = 0;
		robot_actions_msg.forceDetection = false;
		robot_actions_msg.incrementXaxis = 0.0;
		robot_actions_msg.incrementYaxis = -0.008;
		robot_actions_msg.incrementZaxis = 0.0;
		robot_actions_pub.publish(robot_actions_msg);
		while( !robotActionsReady )
		{
			cout << "robotActionReady = " << robotActionsReady << endl;
		}	
		robotActionsReady = false;

       // move robot in x and y position on the blue button
		cout << "robot action: move up to extract battery side" << endl;
        sleep(0.5);
		robot_actions_msg.action = "moveToCartesian";
		robot_actions_msg.position = 0;
		robot_actions_msg.speed = 0;
		robot_actions_msg.force = 0;
		robot_actions_msg.forceDetection = false;
		robot_actions_msg.incrementXaxis = 0.0;
		robot_actions_msg.incrementYaxis = 0.0;
		robot_actions_msg.incrementZaxis = 0.050;
		robot_actions_pub.publish(robot_actions_msg);
		while( !robotActionsReady )
		{
			cout << "robotActionReady = " << robotActionsReady << endl;
		}	
		robotActionsReady = false;


      // move robot in x and y position on the blue button
/*		cout << "robot action: rotate end effector" << endl;
        sleep(0.5);
		robot_actions_msg.action = "moveToJoints";
		robot_actions_msg.position = 0;
		robot_actions_msg.speed = 0;
		robot_actions_msg.force = 0;
		robot_actions_msg.forceDetection = false;
		robot_actions_msg.incrementXaxis = 0.0;
		robot_actions_msg.incrementYaxis = 0.0;
		robot_actions_msg.incrementZaxis = 0.0;
		robot_actions_msg.robotJoints[0] = 0.0;
		robot_actions_msg.robotJoints[1] = 0.0;
		robot_actions_msg.robotJoints[2] = 0.0;
		robot_actions_msg.robotJoints[3] = 0.0;
		robot_actions_msg.robotJoints[4] = 0.0;
		robot_actions_msg.robotJoints[5] = -90.0;
		robot_actions_pub.publish(robot_actions_msg);
		while( !robotActionsReady )
		{
			cout << "robotActionReady = " << robotActionsReady << endl;
		}	
		robotActionsReady = false;
*/

		// close gripper to sepcific distance to extract batteries
		cout << "gripper action: close gripper to slide battery" << endl;
		gripper_actions_msg.action = "close";
		gripper_actions_msg.position = 220;
		gripper_actions_msg.speed = 200;
		gripper_actions_msg.force = 150;
		gripper_actions_msg.useContactDetection = true;
		gripper_actions_pub.publish(gripper_actions_msg);
		while( !gripperActionsReady )
		{
			cout << "gripperActionReady = " << gripperActionsReady << endl;
		}	
		gripperActionsReady = false;
		sleep(1);

      // move robot in x and y position on the blue button
		cout << "robot action: rotate end effector to slide battery" << endl;
        sleep(0.5);
		robot_actions_msg.action = "moveToJoints";
		robot_actions_msg.position = 0;
		robot_actions_msg.speed = 0;
		robot_actions_msg.force = 0;
		robot_actions_msg.forceDetection = false;
		robot_actions_msg.incrementXaxis = 0.0;
		robot_actions_msg.incrementYaxis = 0.0;
		robot_actions_msg.incrementZaxis = 0.0;
		robot_actions_msg.robotJoints[0] = 0.0;
		robot_actions_msg.robotJoints[1] = 0.0;
		robot_actions_msg.robotJoints[2] = 0.0;
		robot_actions_msg.robotJoints[3] = 0.0;
		robot_actions_msg.robotJoints[4] = 0.0;
		robot_actions_msg.robotJoints[5] = -180.0;
		robot_actions_pub.publish(robot_actions_msg);
		while( !robotActionsReady )
		{
			cout << "robotActionReady = " << robotActionsReady << endl;
		}	
		robotActionsReady = false;


       // move robot in x and y position on the blue button
		cout << "robot action: move to edge of battery for sliding it" << endl;
        sleep(0.5);
		robot_actions_msg.action = "moveToCartesian";
		robot_actions_msg.position = 0;
		robot_actions_msg.speed = 0;
		robot_actions_msg.force = 0;
		robot_actions_msg.forceDetection = false;
		robot_actions_msg.incrementXaxis = 0.030;
		robot_actions_msg.incrementYaxis = 0.030;
		robot_actions_msg.incrementZaxis = 0.0;
		robot_actions_pub.publish(robot_actions_msg);
		while( !robotActionsReady )
		{
			cout << "robotActionReady = " << robotActionsReady << endl;
		}	
		robotActionsReady = false;


       // move robot in x and y position on the blue button
		cout << "robot action: move down to detect battery for sliding it" << endl;
        sleep(0.5);
		robot_actions_msg.action = "moveToCartesian";
		robot_actions_msg.position = 0;
		robot_actions_msg.speed = 0;
		robot_actions_msg.force = 0;
		robot_actions_msg.forceDetection = true;
		robot_actions_msg.incrementXaxis = 0.0;
		robot_actions_msg.incrementYaxis = 0.0;
		robot_actions_msg.incrementZaxis = -0.078;
		robot_actions_pub.publish(robot_actions_msg);
		while( !robotActionsReady )
		{
			cout << "robotActionReady = " << robotActionsReady << endl;
		}	
		robotActionsReady = false;


       // move robot in x and y position on the blue button
		cout << "robot action: slide battery for extraction" << endl;
        sleep(0.5);
		robot_actions_msg.action = "moveToCartesian";
		robot_actions_msg.position = 0;
		robot_actions_msg.speed = 0;
		robot_actions_msg.force = 0;
		robot_actions_msg.forceDetection = false;
		robot_actions_msg.incrementXaxis = 0.0;
		robot_actions_msg.incrementYaxis = 0.025;
		robot_actions_msg.incrementZaxis = 0.0;
		robot_actions_pub.publish(robot_actions_msg);
		while( !robotActionsReady )
		{
			cout << "robotActionReady = " << robotActionsReady << endl;
		}	
		robotActionsReady = false;

       // move robot in x and y position on the blue button
		cout << "robot action: move robot up for extraction" << endl;
        sleep(0.5);
		robot_actions_msg.action = "moveToCartesian";
		robot_actions_msg.position = 0;
		robot_actions_msg.speed = 0;
		robot_actions_msg.force = 0;
		robot_actions_msg.forceDetection = false;
		robot_actions_msg.incrementXaxis = 0.0;
		robot_actions_msg.incrementYaxis = 0.0;
		robot_actions_msg.incrementZaxis = 0.040;
		robot_actions_pub.publish(robot_actions_msg);
		while( !robotActionsReady )
		{
			cout << "robotActionReady = " << robotActionsReady << endl;
		}	
		robotActionsReady = false;

		// send robot to home position
		cout << "robot action: moveToHomePosition" << endl;
        sleep(0.5);
		robot_actions_msg.action = "moveToHomePosition";
		robot_actions_pub.publish(robot_actions_msg);
		while( !robotActionsReady )
		{
			cout << "robotActionReady = " << robotActionsReady << endl;
		}	
		robotActionsReady = false;

        // End of task 4: remove battery case and extract batteries
        /****************************************************/
	}

    usleep(1000);
    ros::waitForShutdown();

    return 0;
}
