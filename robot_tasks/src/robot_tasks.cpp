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

#include <Eigen/Dense>


#define PI 3.14159265


using namespace std;
using namespace Eigen;


bool gripperActionsReady = false;
bool robotActionsReady = false;

float boxAngle = 0.0;


float moveTransformX = 0.0;
float moveTransformY = 0.0;

void funcTransformPosition(float boxAngleValue, float currentXPos, float currentYPos, float nextXPos, float nextYPos)
{
	MatrixXd translationMatrix(3,3);
	MatrixXd rotationMatrix(3,3);
	MatrixXd currentLocation(3,1);
	MatrixXd newLocation(3,1);

	float boxAngleValueInRad = boxAngleValue * PI / 180.0;

	translationMatrix << 1, 0, nextXPos,
						 0, 1, nextYPos,
						 0, 0, 1;

	currentLocation << currentXPos, currentYPos, 1;

	currentLocation = translationMatrix * currentLocation;

	rotationMatrix << cos(boxAngleValueInRad), -sin(boxAngleValueInRad), 0,
				      sin(boxAngleValueInRad),  cos(boxAngleValueInRad), 0,
						 0, 0, 1;

	newLocation = rotationMatrix * currentLocation;

	moveTransformX = newLocation(0);
	moveTransformY = newLocation(1);
}


// callback function from gripper actions status
void gripperActionsStatusCallback(const std_msgs::Bool::ConstPtr& msg)
{
	gripperActionsReady = msg->data;
}

// callback function from robot actions status
void robotActionsStatusCallback(const std_msgs::Bool::ConstPtr& msg)
{
	robotActionsReady = msg->data;
}

// callback function from box modules
void boxStatusCallback(const std_msgs::Float32::ConstPtr& msg)
{
	boxAngle = msg->data;
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
    ros::Subscriber box_status_sub = node_handle.subscribe("box_status", 1, boxStatusCallback);


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
//    set_speed_frac.request.speed_slider_fraction = 0.50; // change velocity in the slider of teach pendant
//    clientSpeedSlider.call(set_speed_frac);   

    geometry_msgs::Pose ReleasePosBattery1; 
    geometry_msgs::Pose ReleasePosEthernet; 

	// reset the gripper using default values
	cout << "gripper action: empty" << endl;
	gripper_actions_msg.action = "empty";
	gripper_actions_msg.position = 0;
	gripper_actions_msg.speed = 0;
	gripper_actions_msg.force = 0;
	gripper_actions_msg.useContactDetection = 0;
	cout << "gripper action: reset2" << endl;
	gripper_actions_pub.publish(gripper_actions_msg);
//	sleep(1);
	sleep(0.5);

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
//		sleep(0.5);
    
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
//		sleep(0.5);

        cout << "**********************************" << endl;
        cout << "select task" << endl;
        cout << "0 - align end-effector to box" << endl;
        cout << "1 - blue button" << endl;
        cout << "2 - AA batteries " << endl;
        cout << "3 - coin battery" << endl;
        cout << "4 - ethernet cable" << endl;
        cout << "5 - key" << endl;
        cout << "6 - exit" << endl;

        int taskNumber = 0;
        cin >> taskNumber;


        if( taskNumber == 0 )
        {
        /****************************************************/
        // Begin of task 0: set robot end-effector orientation to box orientation

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
        
        geometry_msgs::Pose homePosition = move_group.getCurrentPose().pose;

        cout << "Angle box received = " << boxAngle << endl;
        int a = 0;
        cout << "press a key to continue" << endl;
        cin >> a;

		cout << "robot action: taskHomePosition" << endl;
        sleep(0.5);
		robot_actions_msg.action = "taskHomePosition";
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
		robot_actions_msg.robotJoints[5] = boxAngle;    // end-effector angle
		robot_actions_pub.publish(robot_actions_msg);
		while( !robotActionsReady )
		{
			cout << "robotActionReady = " << robotActionsReady << endl;
		}	
		robotActionsReady = false;

        // End of task 0: set robot end-effector orientation to box orientation
        /****************************************************/
        }
        if( taskNumber == 1 )
        {
        /****************************************************/
        // Begining of task 1: box localisation and press blue button

		// send robot to home position
        geometry_msgs::Pose homePosition = move_group.getCurrentPose().pose;

		cout << "robot action: moveToHomePosition" << endl;
        sleep(0.5);
		robot_actions_msg.action = "moveToHomePosition";
		robot_actions_pub.publish(robot_actions_msg);
    	while( !robotActionsReady )
		{
			cout << "robotActionReady = " << robotActionsReady << endl;
		}	
		robotActionsReady = false;
        
        homePosition = move_group.getCurrentPose().pose;

        cout << "Angle box received = " << boxAngle << endl;
        int a = 0;
        cout << "press a key to continue" << endl;
        cin >> a;

		cout << "robot action: taskHomePosition" << endl;
        sleep(0.5);
		robot_actions_msg.action = "taskHomePosition";
		robot_actions_msg.robotJoints[5] = boxAngle;    // end-effector angle
		robot_actions_pub.publish(robot_actions_msg);
		while( !robotActionsReady )
		{
			cout << "robotActionReady = " << robotActionsReady << endl;
		}	
		robotActionsReady = false;

        homePosition = move_group.getCurrentPose().pose;

        float blueButtonX = 0.23359;
        float blueButtonY = 0.08075;
   
        // move robot in x and y position on the blue button
		cout << "robot action: move to x and y positions" << endl;
        sleep(0.5);
		robot_actions_msg.action = "moveToCartesian";
		robot_actions_msg.position = 0;
		robot_actions_msg.speed = 0;
		robot_actions_msg.force = 0;
		robot_actions_msg.forceDetection = false;
		robot_actions_msg.incrementXaxis = blueButtonX - homePosition.position.x;
		robot_actions_msg.incrementYaxis = blueButtonY - homePosition.position.y; 
		robot_actions_msg.incrementZaxis = 0.0;
		robot_actions_pub.publish(robot_actions_msg);
		while( !robotActionsReady )
		{
			cout << "robotActionReady = " << robotActionsReady << endl;
		}	
		robotActionsReady = false;

/*        float blueButtonX = 0.22321;
        float blueButtonY = 0.09197;
   
        // move robot in x and y position on the blue button
		cout << "robot action: move to x and y positions" << endl;
        sleep(0.5);
		robot_actions_msg.action = "moveToCartesian";
		robot_actions_msg.position = 0;
		robot_actions_msg.speed = 0;
		robot_actions_msg.force = 0;
		robot_actions_msg.forceDetection = false;
		robot_actions_msg.incrementXaxis = blueButtonX - homePosition.position.x;
		robot_actions_msg.incrementYaxis = blueButtonY - homePosition.position.y; 
		robot_actions_msg.incrementZaxis = 0.0;
		robot_actions_pub.publish(robot_actions_msg);
		while( !robotActionsReady )
		{
			cout << "robotActionReady = " << robotActionsReady << endl;
		}	
		robotActionsReady = false;
*/

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

        // move robot up on z axis
		cout << "robot action: move up on z axis" << endl;
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

        homePosition = move_group.getCurrentPose().pose;

        cout << "Angle box received = " << boxAngle << endl;
        a = 0;
        cout << "press a key to continue" << endl;
        cin >> a;

		cout << "robot action: taskHomePosition" << endl;
        sleep(0.5);
		robot_actions_msg.action = "taskHomePosition";
		robot_actions_msg.robotJoints[5] = boxAngle;    // end-effector angle
		robot_actions_pub.publish(robot_actions_msg);
		while( !robotActionsReady )
		{
			cout << "robotActionReady = " << robotActionsReady << endl;
		}	
		robotActionsReady = false;

        // End of task 1: localisation and press blue button
        /****************************************************/
        }
        else if( taskNumber == 2 )
        {
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

        float batteryLidX = 0.25173;
        float batteryLidY = -0.01245; // 0.01033 is added to move the robot on the lid
        geometry_msgs::Pose homePosition = move_group.getCurrentPose().pose;

       // move robot in x and y position on the lid
		cout << "robot action: move to x and y positions on lid" << endl;
        sleep(0.5);
		robot_actions_msg.action = "moveToCartesian";
		robot_actions_msg.position = 0;
		robot_actions_msg.speed = 0;
		robot_actions_msg.force = 0;
		robot_actions_msg.forceDetection = false;
		robot_actions_msg.incrementXaxis = batteryLidX - homePosition.position.x;
		robot_actions_msg.incrementYaxis = batteryLidY - homePosition.position.y;
		robot_actions_msg.incrementZaxis = 0.0;
		robot_actions_pub.publish(robot_actions_msg);
		while( !robotActionsReady )
		{
			cout << "robotActionReady = " << robotActionsReady << endl;
		}	
		robotActionsReady = false;

        geometry_msgs::Pose centerLid = move_group.getCurrentPose().pose;
        float batteryLidSlideX = centerLid.position.x;
        float batteryLidSlideY = 0.01033; // 0.01033 is added to move the robot on the lid
  
 // move robot in x and y position on the lid
		cout << "robot action: move to x and y positions on lid" << endl;
        sleep(0.5);
		robot_actions_msg.action = "moveToCartesian";
		robot_actions_msg.position = 0;
		robot_actions_msg.speed = 0;
		robot_actions_msg.force = 0;
		robot_actions_msg.forceDetection = false;
		robot_actions_msg.incrementXaxis = batteryLidSlideX - centerLid.position.x;
		robot_actions_msg.incrementYaxis = batteryLidSlideY - centerLid.position.y;
		robot_actions_msg.incrementZaxis = 0.0;
		robot_actions_pub.publish(robot_actions_msg);
		while( !robotActionsReady )
		{
			cout << "robotActionReady = " << robotActionsReady << endl;
		}	
		robotActionsReady = false;



//int b=0;
//cin>>b;

       // move robot down and touch the battery lid
		cout << "robot action: move down, detect force" << endl;
        sleep(0.5);
		robot_actions_msg.action = "moveToCartesian";
		robot_actions_msg.position = 0;
		robot_actions_msg.speed = 0;
		robot_actions_msg.force = 0;
		robot_actions_msg.forceDetection = true;
		robot_actions_msg.incrementXaxis = 0.0;
		robot_actions_msg.incrementYaxis = 0.0;
		robot_actions_msg.incrementZaxis = -0.075;
		robot_actions_pub.publish(robot_actions_msg);
		while( !robotActionsReady )
		{
			cout << "robotActionReady = " << robotActionsReady << endl;
		}	
		robotActionsReady = false;

       // move robot on z axis down and touch sligtly to slide the lid
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

       // move robot on y axis and slide the lid
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

       // move robot o z axis
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
 
      // move robot's last joint to prepare the gripper for grasping action
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
//		sleep(0.5);

       // move robot on z axis down
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

		// close the gripper without force detection, grasp the lid
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
//		sleep(0.5);

       // move robot up
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

       // move robot in x and y position to leave the lid on the table
		cout << "robot action: move to release (lid) position" << endl;
        sleep(0.5);
		robot_actions_msg.action = "moveToCartesian";
		robot_actions_msg.position = 0;
		robot_actions_msg.speed = 0;
		robot_actions_msg.force = 0;
		robot_actions_msg.forceDetection = false;
		robot_actions_msg.incrementXaxis = -0.100;
		robot_actions_msg.incrementYaxis = -0.060;
		robot_actions_msg.incrementZaxis = 0.0;
		robot_actions_pub.publish(robot_actions_msg);
		while( !robotActionsReady )
		{
			cout << "robotActionReady = " << robotActionsReady << endl;
		}	
		robotActionsReady = false;

    // move robot in z position
		cout << "robot action: move to release (lid) position" << endl;
        sleep(0.5);
		robot_actions_msg.action = "moveToCartesian";
		robot_actions_msg.position = 0;
		robot_actions_msg.speed = 0;
		robot_actions_msg.force = 0;
		robot_actions_msg.forceDetection = false;
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
		gripper_actions_msg.position = 130;
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
//		sleep(0.5);

       // move robot up, on z axis before moving to home position to avoid collision to the box
		cout << "robot action: move to release (lid) position" << endl;
        sleep(0.5);
		robot_actions_msg.action = "moveToCartesian";
		robot_actions_msg.position = 0;
		robot_actions_msg.speed = 0;
		robot_actions_msg.force = 0;
		robot_actions_msg.forceDetection = false;
		robot_actions_msg.incrementXaxis = 0.0;
		robot_actions_msg.incrementYaxis = 0.0;
		robot_actions_msg.incrementZaxis = 0.090;
		robot_actions_pub.publish(robot_actions_msg);
		while( !robotActionsReady )
		{
			cout << "robotActionReady = " << robotActionsReady << endl;
		}	
		robotActionsReady = false;

    int signBattery=1;

    for (int batteryNumber=0; batteryNumber<2; batteryNumber++){
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

        homePosition = move_group.getCurrentPose().pose;
    // move robot to the top of the lid on x and y axis,
		cout << "robot action: move to x and y positions on lid" << endl;
        sleep(0.5);
		robot_actions_msg.action = "moveToCartesian";
		robot_actions_msg.position = 0;
		robot_actions_msg.speed = 0;
		robot_actions_msg.force = 0;
		robot_actions_msg.forceDetection = false;
		robot_actions_msg.incrementXaxis = centerLid.position.x - homePosition.position.x;
		robot_actions_msg.incrementYaxis = centerLid.position.y - homePosition.position.y;
		robot_actions_msg.incrementZaxis = 0.0;
		robot_actions_pub.publish(robot_actions_msg);
		while( !robotActionsReady )
		{
			cout << "robotActionReady = " << robotActionsReady << endl;
		}	
		robotActionsReady = false;


    // move robot to the top of the lid on x and y axis,
		cout << "robot action: move to x and y positions on lid" << endl;
        sleep(0.5);
		robot_actions_msg.action = "moveToCartesian";
		robot_actions_msg.position = 0;
		robot_actions_msg.speed = 0;
		robot_actions_msg.force = 0;
		robot_actions_msg.forceDetection = false;
		robot_actions_msg.incrementXaxis = 0.006 * signBattery;
        if (batteryNumber > 0 )
		robot_actions_msg.incrementYaxis = 0.008 * signBattery;
        else
        robot_actions_msg.incrementYaxis = 0.015 * signBattery;
		robot_actions_msg.incrementZaxis = 0.0;
		robot_actions_pub.publish(robot_actions_msg);
		while( !robotActionsReady )
		{
			cout << "robotActionReady = " << robotActionsReady << endl;
		}	
		robotActionsReady = false;

		// open gripper to sepcific distance to extract batteries
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
//		sleep(0.5);

    if (batteryNumber > 0){
        // rotate the robot's last joint before starting the sliding battery to the vertical position.
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

        } 
       // move robot on z axis down, and detect the batteries
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

       // move robot down and place the plastic tip in the edge of the battery case
		cout << "robot action: move down to detect battery position" << endl;
        sleep(0.5);
		robot_actions_msg.action = "moveToCartesian";
		robot_actions_msg.position = 0;
		robot_actions_msg.speed = 0;
		robot_actions_msg.force = 0;
		robot_actions_msg.forceDetection = false;
		robot_actions_msg.incrementXaxis = 0.0;
		robot_actions_msg.incrementYaxis = 0.0;
		robot_actions_msg.incrementZaxis = -0.008;
		robot_actions_pub.publish(robot_actions_msg);
		while( !robotActionsReady )
		{
			cout << "robotActionReady = " << robotActionsReady << endl;
		}	
		robotActionsReady = false;

       // move robot on y axis to remove the first battery
		cout << "robot action: slide to press battery side" << endl;
        sleep(0.5);
		robot_actions_msg.action = "moveToCartesian";
		robot_actions_msg.position = 0;
		robot_actions_msg.speed = 0;
		robot_actions_msg.force = 0;
		robot_actions_msg.forceDetection = false;
		robot_actions_msg.incrementXaxis = 0.0;
		robot_actions_msg.incrementYaxis = -0.008 * signBattery;
		robot_actions_msg.incrementZaxis = 0.0;
		robot_actions_pub.publish(robot_actions_msg);
		while( !robotActionsReady )
		{
			cout << "robotActionReady = " << robotActionsReady << endl;
		}	
		robotActionsReady = false;

       // move robot up on z axis
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
//		sleep(0.5);

      // rotate the robot's last joint before starting the sliding battery to the vertical position.
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
		robot_actions_msg.robotJoints[5] = -180.0 * signBattery;
		robot_actions_pub.publish(robot_actions_msg);
		while( !robotActionsReady )
		{
			cout << "robotActionReady = " << robotActionsReady << endl;
		}	
		robotActionsReady = false;


       // move robot on x and y position to the end of the battery1
		cout << "robot action: move to edge of battery for sliding it" << endl;
        sleep(0.5);
		robot_actions_msg.action = "moveToCartesian";
		robot_actions_msg.position = 0;
		robot_actions_msg.speed = 0;
		robot_actions_msg.force = 0;
		robot_actions_msg.forceDetection = false;
		robot_actions_msg.incrementXaxis = 0.0;
        if (batteryNumber > 0)
		robot_actions_msg.incrementYaxis = -0.028 * signBattery;
        else
        robot_actions_msg.incrementYaxis = -0.035 * signBattery;
		robot_actions_msg.incrementZaxis = 0.0;
		robot_actions_pub.publish(robot_actions_msg);
		while( !robotActionsReady )
		{
			cout << "robotActionReady = " << robotActionsReady << endl;
		}	
		robotActionsReady = false;


       // move robot down
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


       // move robot down
        if( batteryNumber > 0 )
        {
		    cout << "robot action: move down to detect second battery for sliding it with no force" << endl;
            sleep(0.5);
		    robot_actions_msg.action = "moveToCartesian";
		    robot_actions_msg.position = 0;
		    robot_actions_msg.speed = 0;
		    robot_actions_msg.force = 0;
		    robot_actions_msg.forceDetection = false;
		    robot_actions_msg.incrementXaxis = 0.0;
		    robot_actions_msg.incrementYaxis = 0.0;
		    robot_actions_msg.incrementZaxis = -0.003;
		    robot_actions_pub.publish(robot_actions_msg);
		    while( !robotActionsReady )
		    {
			    cout << "robotActionReady = " << robotActionsReady << endl;
		    }	
		    robotActionsReady = false;
        }

       // move robot on y axis to position the battery vertically
		cout << "robot action: slide battery for extraction" << endl;
        sleep(0.5);
		robot_actions_msg.action = "moveToCartesian";
		robot_actions_msg.position = 0;
		robot_actions_msg.speed = 0;
		robot_actions_msg.force = 0;
		robot_actions_msg.forceDetection = false;
		robot_actions_msg.incrementXaxis = 0.0;
        
        if( batteryNumber > 0 )
    		robot_actions_msg.incrementYaxis = 0.035 * signBattery;
        else
    		robot_actions_msg.incrementYaxis = 0.030 * signBattery;

		robot_actions_msg.incrementZaxis = 0.0;
		robot_actions_pub.publish(robot_actions_msg);
		while( !robotActionsReady )
		{
			cout << "robotActionReady = " << robotActionsReady << endl;
		}	
		robotActionsReady = false;

       // move robot in 
		cout << "robot action: move robot on y for extraction" << endl;
        sleep(0.5);
		robot_actions_msg.action = "moveToCartesian";
		robot_actions_msg.position = 0;
		robot_actions_msg.speed = 0;
		robot_actions_msg.force = 0;
		robot_actions_msg.forceDetection = false;
		robot_actions_msg.incrementXaxis = 0.0;
		robot_actions_msg.incrementYaxis = -0.010 * signBattery;
		robot_actions_msg.incrementZaxis = 0.0;
		robot_actions_pub.publish(robot_actions_msg);
		while( !robotActionsReady )
		{
			cout << "robotActionReady = " << robotActionsReady << endl;
		}	
		robotActionsReady = false;

       // move robot in 
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

    // open gripper to sepcific distance to extract batteries
		cout << "gripper action: open gripper to grasp battery" << endl;
		gripper_actions_msg.action = "open";
        if( batteryNumber > 0 )
    		gripper_actions_msg.position = 130;
        else
	    	gripper_actions_msg.position = 130;
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
//		sleep(0.5);


    // move robot on y axis
		cout << "robot action: move on y to grasp battery" << endl;
        sleep(0.5);
		robot_actions_msg.action = "moveToCartesian";
		robot_actions_msg.position = 0;
		robot_actions_msg.speed = 0;
		robot_actions_msg.force = 0;
		robot_actions_msg.forceDetection = false;
		robot_actions_msg.incrementXaxis = 0.0;
		robot_actions_msg.incrementYaxis = 0.025 * signBattery;
		robot_actions_msg.incrementZaxis = 0.0;
		robot_actions_pub.publish(robot_actions_msg);
		while( !robotActionsReady )
		{
			cout << "robotActionReady = " << robotActionsReady << endl;
		}	
		robotActionsReady = false;


    // move robot down to grasp the battery
		cout << "robot action: move down to grasp battery" << endl;
        sleep(0.5);
		robot_actions_msg.action = "moveToCartesian";
		robot_actions_msg.position = 0;
		robot_actions_msg.speed = 0;
		robot_actions_msg.force = 0;
		robot_actions_msg.forceDetection = false;
		robot_actions_msg.incrementXaxis = 0.0;
		robot_actions_msg.incrementYaxis = 0.0;

        if( batteryNumber > 0 )
    		robot_actions_msg.incrementZaxis = -0.040;
        else
            robot_actions_msg.incrementZaxis = -0.030;

		robot_actions_pub.publish(robot_actions_msg);
		while( !robotActionsReady )
		{
			cout << "robotActionReady = " << robotActionsReady << endl;
		}	
		robotActionsReady = false;


        if( batteryNumber > 0 )
        {
        // open gripper to sepcific distance to extract batteries
		    cout << "gripper action: open gripper to grasp battery" << endl;
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
        }

       // move robot down
        if( batteryNumber > 0 )
        {
		    cout << "robot action: move down to detect second battery for sliding it with no force" << endl;
            sleep(0.5);
		    robot_actions_msg.action = "moveToCartesian";
		    robot_actions_msg.position = 0;
		    robot_actions_msg.speed = 0;
		    robot_actions_msg.force = 0;
		    robot_actions_msg.forceDetection = false;
		    robot_actions_msg.incrementXaxis = 0.0;
		    robot_actions_msg.incrementYaxis = 0.030;
		    robot_actions_msg.incrementZaxis = 0.0;
		    robot_actions_pub.publish(robot_actions_msg);
		    while( !robotActionsReady )
		    {
			    cout << "robotActionReady = " << robotActionsReady << endl;
		    }	
		    robotActionsReady = false;
        }


    // close gripper to sepcific distance to extract batteries
		cout << "gripper action: close gripper to grasp battery" << endl;
		gripper_actions_msg.action = "close";
		gripper_actions_msg.position = 225;
		gripper_actions_msg.speed = 50;
		gripper_actions_msg.force = 220;
		gripper_actions_msg.useContactDetection = true;
		gripper_actions_pub.publish(gripper_actions_msg);
		while( !gripperActionsReady )
		{
			cout << "gripperActionReady = " << gripperActionsReady << endl;
		}	
		gripperActionsReady = false;
		sleep(1);
//		sleep(0.5);

    // move robot on z axis
		cout << "robot action: move robot up for extraction" << endl;
        sleep(0.5);
		robot_actions_msg.action = "moveToCartesian";
		robot_actions_msg.position = 0;
		robot_actions_msg.speed = 0;
		robot_actions_msg.force = 0;
		robot_actions_msg.forceDetection = false;
		robot_actions_msg.incrementXaxis = 0.0;
		robot_actions_msg.incrementYaxis = 0.0;
        if (batteryNumber > 0)
		robot_actions_msg.incrementZaxis = 0.060;
        else
        robot_actions_msg.incrementZaxis = 0.040;
		robot_actions_pub.publish(robot_actions_msg);
		while( !robotActionsReady )
		{
			cout << "robotActionReady = " << robotActionsReady << endl;
		}	
		robotActionsReady = false;


        // rotate the robot's last joint to insert battery
		cout << "robot action: rotate end effector to insert battery" << endl;
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
        if( batteryNumber > 0 )
    		robot_actions_msg.robotJoints[5] = -90.0; // * signBattery;
        else
    		robot_actions_msg.robotJoints[5] = -45.0; // * signBattery;

		robot_actions_pub.publish(robot_actions_msg);
		while( !robotActionsReady )
		{
			cout << "robotActionReady = " << robotActionsReady << endl;
		}	
		robotActionsReady = false;


        bool batteryReleasedReady = false;
        
        float holeBatteryPosX = 0.2578;// 0.25781
        float holeBatteryPosY = 0.0603; // 0.05876
        float incX = 0.0;
        float incY = 0.0;
        if (batteryNumber > 0){
            holeBatteryPosX = ReleasePosBattery1.position.x + 0.0;
            holeBatteryPosY = ReleasePosBattery1.position.y + 0.021;
           }

        cout << "Initial batteryReleasedReady = " << batteryReleasedReady << endl;
        int a = 0;

        while( batteryReleasedReady == false )
        {
            cout << "In while loop, press to continue" << endl;
            cin >>  a;

            geometry_msgs::Pose graspBatteryPos = move_group.getCurrentPose().pose;

            holeBatteryPosX = holeBatteryPosX + incX;
            holeBatteryPosY = holeBatteryPosY + incY;

         // move robot on y axis to the battery hole
		    cout << "robot action: move robot to hole to insert battery" << endl;
            sleep(0.5);
		    robot_actions_msg.action = "moveToCartesian";
		    robot_actions_msg.position = 0;
		    robot_actions_msg.speed = 0;
		    robot_actions_msg.force = 0;
		    robot_actions_msg.forceDetection = false;
		    robot_actions_msg.incrementXaxis = holeBatteryPosX - graspBatteryPos.position.x;
		    robot_actions_msg.incrementYaxis = holeBatteryPosY - graspBatteryPos.position.y;
		    robot_actions_msg.incrementZaxis = 0.0;
		    robot_actions_pub.publish(robot_actions_msg);
		    while( !robotActionsReady )
		    {
			    cout << "robotActionReady = " << robotActionsReady << endl;
		    }	
		    robotActionsReady = false;  

            graspBatteryPos = move_group.getCurrentPose().pose;
            cout << "Current x, y, z position: " << graspBatteryPos.position.x << ", " << graspBatteryPos.position.y << ", " << graspBatteryPos.position.z << endl;
            cout << "Increment x, y position: " << incX << ", " << incX << endl;

            // move robot on z axis down to the battery hole
		    cout << "robot action: move robot down to insert battery" << endl;
            sleep(0.5);
		    robot_actions_msg.action = "moveToCartesian";
		    robot_actions_msg.position = 0;
		    robot_actions_msg.speed = 0;
		    robot_actions_msg.force = 0;
		    robot_actions_msg.forceDetection = true;
		    robot_actions_msg.incrementXaxis = 0.0;
		    robot_actions_msg.incrementYaxis = 0.0;
		    robot_actions_msg.incrementZaxis = -0.070;

            cout << "Target x, y, z position: " << graspBatteryPos.position.x << ", " << graspBatteryPos.position.y << ", " << graspBatteryPos.position.z - 0.070 << endl;
            cout << "press to continue" << endl;
            cin >> a;


		    robot_actions_pub.publish(robot_actions_msg);
		    while( !robotActionsReady )
		    {
			    cout << "robotActionReady = " << robotActionsReady << endl;
		    }	
		    robotActionsReady = false;
            
            geometry_msgs::Pose releasePositionCheck = move_group.getCurrentPose().pose;

            cout << "Released position check: " << releasePositionCheck << endl;
            cout << "Center lid: " << centerLid << endl;

            cout << "press to continue" << endl;
            cin >> a;

            if( releasePositionCheck.position.z <= 0.42){
                batteryReleasedReady = true;
                ReleasePosBattery1 = move_group.getCurrentPose().pose;
                }            
            else
                batteryReleasedReady = false;


            cout << "Current batteryReleasedReady = " << batteryReleasedReady << endl;
            cout << "press to continue" << endl;
            cin >> a;

            if( batteryReleasedReady == false )
            {
                // move robot on z axis down to the battery hole
		        cout << "robot action: move robot up to insert battery" << endl;
                sleep(0.5);
		        robot_actions_msg.action = "moveToCartesian";
		        robot_actions_msg.position = 0;
		        robot_actions_msg.speed = 0;
		        robot_actions_msg.force = 0;
		        robot_actions_msg.forceDetection = false;
		        robot_actions_msg.incrementXaxis = 0.0;
		        robot_actions_msg.incrementYaxis = 0.0;
		        robot_actions_msg.incrementZaxis = 0.010;
		        robot_actions_pub.publish(robot_actions_msg);
		        while( !robotActionsReady )
		        {
			        cout << "robotActionReady = " << robotActionsReady << endl;
		        }	
		        robotActionsReady = false;

                cout << "Input increment in X: " << endl;
                cin >> incX;
                cout << "Input increment in Y: " << endl;
                cin >> incY;
            }
                
        }
/*
        geometry_msgs::Pose graspBatteryPos = move_group.getCurrentPose().pose;
        float holeBatteryPosX = 0.2578;// 0.25781
        float holeBatteryPosY = 0.0603; // 0.05876

     // move robot on y axis to the battery hole
	    cout << "robot action: move robot to hole to insert battery" << endl;
        sleep(0.5);
	    robot_actions_msg.action = "moveToCartesian";
	    robot_actions_msg.position = 0;
	    robot_actions_msg.speed = 0;
	    robot_actions_msg.force = 0;
	    robot_actions_msg.forceDetection = false;
	    robot_actions_msg.incrementXaxis = holeBatteryPosX - graspBatteryPos.position.x;
	    robot_actions_msg.incrementYaxis = holeBatteryPosY - graspBatteryPos.position.y;
	    robot_actions_msg.incrementZaxis = 0.0;
	    robot_actions_pub.publish(robot_actions_msg);
	    while( !robotActionsReady )
	    {
		    cout << "robotActionReady = " << robotActionsReady << endl;
	    }	
	    robotActionsReady = false;  

        // move robot on z axis down to the battery hole
	    cout << "robot action: move robot down to insert battery" << endl;
        sleep(0.5);
	    robot_actions_msg.action = "moveToCartesian";
	    robot_actions_msg.position = 0;
	    robot_actions_msg.speed = 0;
	    robot_actions_msg.force = 0;
	    robot_actions_msg.forceDetection = true;
	    robot_actions_msg.incrementXaxis = 0.0;
	    robot_actions_msg.incrementYaxis = 0.0;
	    robot_actions_msg.incrementZaxis = -0.040;
	    robot_actions_pub.publish(robot_actions_msg);
	    while( !robotActionsReady )
	    {
		    cout << "robotActionReady = " << robotActionsReady << endl;
	    }	
	    robotActionsReady = false;  
*/
/*
        // open gripper to sepcific distance to extract batteries
		cout << "gripper action: open gripper to release battery" << endl;
		gripper_actions_msg.action = "open";
		gripper_actions_msg.position = 185;
		gripper_actions_msg.speed = 50;
		gripper_actions_msg.force = 200;
		gripper_actions_msg.useContactDetection = false;
		gripper_actions_pub.publish(gripper_actions_msg);
		while( !gripperActionsReady )
		{
			cout << "gripperActionReady = " << gripperActionsReady << endl;
		}	
		gripperActionsReady = false;
		sleep(1);
//		sleep(0.5);

        // move robot on z axis down to the battery hole
		cout << "robot action: move robot up" << endl;
        sleep(0.5);
		robot_actions_msg.action = "moveToCartesian";
		robot_actions_msg.position = 0;
		robot_actions_msg.speed = 0;
		robot_actions_msg.force = 0;
		robot_actions_msg.forceDetection = false;
		robot_actions_msg.incrementXaxis = 0.0;
		robot_actions_msg.incrementYaxis = 0.0;
		robot_actions_msg.incrementZaxis = 0.010;
		robot_actions_pub.publish(robot_actions_msg);
		while( !robotActionsReady )
		{
			cout << "robotActionReady = " << robotActionsReady << endl;
		}	
		robotActionsReady = false;  

*/
/*    int signMovement = 1;
    for (int c=0; c<4; c++)
    {
		cout << "robot action: move robot up" << endl;
        sleep(0.5);
		robot_actions_msg.action = "moveToCartesian";
		robot_actions_msg.position = 0;
		robot_actions_msg.speed = 0;
		robot_actions_msg.force = 0;
		robot_actions_msg.forceDetection = false;
		robot_actions_msg.incrementXaxis = 0.0;
		robot_actions_msg.incrementYaxis = 0.005 * signMovement;
		robot_actions_msg.incrementZaxis = 0.0;
		robot_actions_pub.publish(robot_actions_msg);
		while( !robotActionsReady )
		{
			cout << "robotActionReady = " << robotActionsReady << endl;
		}	
		robotActionsReady = false;
        signMovement = -1 * signMovement;
        sleep(0.5);
    }   
*/
 // open gripper to sepcific distance to extract batteries
		cout << "gripper action: open gripper to release battery" << endl;
		gripper_actions_msg.action = "open";
		gripper_actions_msg.position = 130;
		gripper_actions_msg.speed = 50;
		gripper_actions_msg.force = 200;
		gripper_actions_msg.useContactDetection = false;
		gripper_actions_pub.publish(gripper_actions_msg);
		while( !gripperActionsReady )
		{
			cout << "gripperActionReady = " << gripperActionsReady << endl;
		}	
		gripperActionsReady = false;
		sleep(1);
//		sleep(0.5);

        // move robot on z axis down to the battery hole
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
        signBattery = signBattery * -1;
} //end of for loop

        // End of task 4: remove battery case and extract batteries
        /****************************************************/
        }
        else if( taskNumber == 3 )
        {
        /****************************************************/
        // Beginning of task 4: remove coin battery
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

            // rotate the robot's last joint to insert battery
		    cout << "robot action: rotate end effector to insert battery" << endl;
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
       		robot_actions_msg.robotJoints[5] = -180.0; // * signBattery;
		    robot_actions_pub.publish(robot_actions_msg);
		    while( !robotActionsReady )
		    {
			    cout << "robotActionReady = " << robotActionsReady << endl;
		    }	
		    robotActionsReady = false;

            int a = 0;
            cout << "press to continue" << endl;
            cin >> a;

 
            geometry_msgs::Pose homePosition = move_group.getCurrentPose().pose;

            float coinBatteryX = 0.3248;
            float coinBatteryY = 0.08600;

        // move robot to the top of the lid on x and y axis,
		    cout << "robot action: move to x and y to top of coin battery" << endl;
            sleep(0.5);
		    robot_actions_msg.action = "moveToCartesian";
		    robot_actions_msg.position = 0;
		    robot_actions_msg.speed = 0;
		    robot_actions_msg.force = 0;
		    robot_actions_msg.forceDetection = false;
		    robot_actions_msg.incrementXaxis = coinBatteryX - homePosition.position.x;
		    robot_actions_msg.incrementYaxis = coinBatteryY - homePosition.position.y;
		    robot_actions_msg.incrementZaxis = 0.0;
		    robot_actions_pub.publish(robot_actions_msg);
		    while( !robotActionsReady )
		    {
			    cout << "robotActionReady = " << robotActionsReady << endl;
		    }	
		    robotActionsReady = false;

            cout << "press to continue" << endl;
            cin >> a;

//            geometry_msgs::Pose centrOfCointBattery = move_group.getCurrentPose().pose;


           // move robot to the top of the lid on x and y axis,
		    cout << "robot action: move down to detect coin battery with force detection" << endl;
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

            cout << "press to continue" << endl;
            cin >> a;

           // move robot to the top of the lid on x and y axis,
		    cout << "robot action: move up without force detection" << endl;
            sleep(0.5);
		    robot_actions_msg.action = "moveToCartesian";
		    robot_actions_msg.position = 0;
		    robot_actions_msg.speed = 0;
		    robot_actions_msg.force = 0;
		    robot_actions_msg.forceDetection = false;
		    robot_actions_msg.incrementXaxis = 0.0;
		    robot_actions_msg.incrementYaxis = 0.0;
		    robot_actions_msg.incrementZaxis = 0.020;
		    robot_actions_pub.publish(robot_actions_msg);
		    while( !robotActionsReady )
		    {
			    cout << "robotActionReady = " << robotActionsReady << endl;
		    }	
		    robotActionsReady = false;

            cout << "press to continue" << endl;
            cin >> a;

     // open gripper to sepcific distance to extract batteries
		    cout << "gripper action: open gripper to release battery" << endl;
		    gripper_actions_msg.action = "open";
		    gripper_actions_msg.position = 130;
		    gripper_actions_msg.speed = 50;
		    gripper_actions_msg.force = 200;
		    gripper_actions_msg.useContactDetection = true;
		    gripper_actions_pub.publish(gripper_actions_msg);
		    while( !gripperActionsReady )
		    {
			    cout << "gripperActionReady = " << gripperActionsReady << endl;
		    }	
		    gripperActionsReady = false;
		    sleep(1);

            cout << "press to continue" << endl;
            cin >> a;

            homePosition = move_group.getCurrentPose().pose;

            float displacementCoinPositionX = 0.0;
            float displacementCoinPositionY = 0.120;

           // move robot to the top of the lid on x and y axis,
		    cout << "robot action: move down to detect coin battery without force detection" << endl;
            sleep(0.5);
		    robot_actions_msg.action = "moveToCartesian";
		    robot_actions_msg.position = 0;
		    robot_actions_msg.speed = 0;
		    robot_actions_msg.force = 0;
		    robot_actions_msg.forceDetection = false;
		    robot_actions_msg.incrementXaxis = 0.0;
		    robot_actions_msg.incrementYaxis = displacementCoinPositionY - homePosition.position.y;
		    robot_actions_msg.incrementZaxis = 0.0;
		    robot_actions_pub.publish(robot_actions_msg);
		    while( !robotActionsReady )
		    {
			    cout << "robotActionReady = " << robotActionsReady << endl;
		    }	
		    robotActionsReady = false;

            cout << "press to continue" << endl;
            cin >> a;

           // move robot to the top of the lid on x and y axis,
		    cout << "robot action: move down to detect coin battery without force detection" << endl;
            sleep(0.5);
		    robot_actions_msg.action = "moveToCartesian";
		    robot_actions_msg.position = 0;
		    robot_actions_msg.speed = 0;
		    robot_actions_msg.force = 0;
		    robot_actions_msg.forceDetection = false;
		    robot_actions_msg.incrementXaxis = 0.0;
		    robot_actions_msg.incrementYaxis = 0.0;
		    robot_actions_msg.incrementZaxis = -0.030;
		    robot_actions_pub.publish(robot_actions_msg);
		    while( !robotActionsReady )
		    {
			    cout << "robotActionReady = " << robotActionsReady << endl;
		    }	
		    robotActionsReady = false;

            cout << "press to continue" << endl;
            cin >> a;

           // move robot to the top of the lid on x and y axis,
		    cout << "robot action: move on y direction to release coin battery" << endl;
            sleep(0.5);
		    robot_actions_msg.action = "moveToCartesian";
		    robot_actions_msg.position = 0;
		    robot_actions_msg.speed = 0;
		    robot_actions_msg.force = 0;
		    robot_actions_msg.forceDetection = false;
		    robot_actions_msg.incrementXaxis = 0.0;
		    robot_actions_msg.incrementYaxis = -0.018;
		    robot_actions_msg.incrementZaxis = 0.0;
		    robot_actions_pub.publish(robot_actions_msg);
		    while( !robotActionsReady )
		    {
			    cout << "robotActionReady = " << robotActionsReady << endl;
		    }	
		    robotActionsReady = false;

            cout << "press to continue" << endl;
            cin >> a;

            geometry_msgs::Pose releasePositionOfCoinBattery = move_group.getCurrentPose().pose;

           // move robot to the top of the lid on x and y axis,
		    cout << "robot action: move up and centre from coin battery" << endl;
            sleep(0.5);
		    robot_actions_msg.action = "moveToCartesian";
		    robot_actions_msg.position = 0;
		    robot_actions_msg.speed = 0;
		    robot_actions_msg.force = 0;
		    robot_actions_msg.forceDetection = false;
		    robot_actions_msg.incrementXaxis = coinBatteryX - releasePositionOfCoinBattery.position.x;
		    robot_actions_msg.incrementYaxis = coinBatteryY - releasePositionOfCoinBattery.position.y;
		    robot_actions_msg.incrementZaxis = 0.030;
		    robot_actions_pub.publish(robot_actions_msg);
		    while( !robotActionsReady )
		    {
			    cout << "robotActionReady = " << robotActionsReady << endl;
		    }	
		    robotActionsReady = false;

            cout << "press to continue" << endl;
            cin >> a;

            // rotate the robot's last joint to insert battery
		    cout << "robot action: rotate end effector to insert battery" << endl;
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
       		robot_actions_msg.robotJoints[5] = 90.0; // * signBattery;
		    robot_actions_pub.publish(robot_actions_msg);
		    while( !robotActionsReady )
		    {
			    cout << "robotActionReady = " << robotActionsReady << endl;
		    }	
		    robotActionsReady = false;

            cout << "press to continue" << endl;
            cin >> a;

           // move robot to the top of the lid on x and y axis,
		    cout << "robot action: move down to grasp coin battery" << endl;
            sleep(0.5);
		    robot_actions_msg.action = "moveToCartesian";
		    robot_actions_msg.position = 0;
		    robot_actions_msg.speed = 0;
		    robot_actions_msg.force = 0;
		    robot_actions_msg.forceDetection = true;
		    robot_actions_msg.incrementXaxis = 0.0;
		    robot_actions_msg.incrementYaxis = 0.0;
		    robot_actions_msg.incrementZaxis = -0.035;
		    robot_actions_pub.publish(robot_actions_msg);
		    while( !robotActionsReady )
		    {
			    cout << "robotActionReady = " << robotActionsReady << endl;
		    }	
		    robotActionsReady = false;

            cout << "press to continue" << endl;
            cin >> a;

     // open gripper to sepcific distance to extract batteries
		    cout << "gripper action: open gripper to release battery" << endl;
		    gripper_actions_msg.action = "open";
		    gripper_actions_msg.position = 220;
		    gripper_actions_msg.speed = 50;
		    gripper_actions_msg.force = 200;
		    gripper_actions_msg.useContactDetection = true;
		    gripper_actions_pub.publish(gripper_actions_msg);
		    while( !gripperActionsReady )
		    {
			    cout << "gripperActionReady = " << gripperActionsReady << endl;
		    }	
		    gripperActionsReady = false;
		    sleep(1);

            cout << "press to continue" << endl;
            cin >> a;

           // move robot to the top of the lid on x and y axis,
		    cout << "robot action: move up to release coin battery" << endl;
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

            cout << "press to continue" << endl;
            cin >> a;

            releasePositionOfCoinBattery = move_group.getCurrentPose().pose;

            float releasedCoinBatteryPositionX = 0.135;
            float releasedCoinBatteryPositionY = -0.140;

           // move robot to the top of the lid on x and y axis,
		    cout << "robot action: move to release position of objects" << endl;
            sleep(0.5);
		    robot_actions_msg.action = "moveToCartesian";
		    robot_actions_msg.position = 0;
		    robot_actions_msg.speed = 0;
		    robot_actions_msg.force = 0;
		    robot_actions_msg.forceDetection = false;
    		robot_actions_msg.incrementXaxis = releasedCoinBatteryPositionX - releasePositionOfCoinBattery.position.x;
    		robot_actions_msg.incrementYaxis = releasedCoinBatteryPositionY - releasePositionOfCoinBattery.position.y;
//    		robot_actions_msg.incrementXaxis = -0.100;
//    		robot_actions_msg.incrementYaxis = -0.060;
		    robot_actions_msg.incrementZaxis = 0.0;
		    robot_actions_pub.publish(robot_actions_msg);
		    while( !robotActionsReady )
		    {
			    cout << "robotActionReady = " << robotActionsReady << endl;
		    }	
		    robotActionsReady = false;

            cout << "press to continue" << endl;
            cin >> a;

           // move robot to the top of the lid on x and y axis,
		    cout << "robot action: move to release position of objects" << endl;
            sleep(0.5);
		    robot_actions_msg.action = "moveToCartesian";
		    robot_actions_msg.position = 0;
		    robot_actions_msg.speed = 0;
		    robot_actions_msg.force = 0;
		    robot_actions_msg.forceDetection = false;
    		robot_actions_msg.incrementXaxis = 0.0;
    		robot_actions_msg.incrementYaxis = 0.0;
		    robot_actions_msg.incrementZaxis = -0.090;
		    robot_actions_pub.publish(robot_actions_msg);
		    while( !robotActionsReady )
		    {
			    cout << "robotActionReady = " << robotActionsReady << endl;
		    }	
		    robotActionsReady = false;

            cout << "press to continue" << endl;
            cin >> a;

         // open gripper to sepcific distance to extract batteries
		    cout << "gripper action: open gripper to release battery" << endl;
		    gripper_actions_msg.action = "open";
		    gripper_actions_msg.position = 130;
		    gripper_actions_msg.speed = 50;
		    gripper_actions_msg.force = 200;
		    gripper_actions_msg.useContactDetection = false;
		    gripper_actions_pub.publish(gripper_actions_msg);
		    while( !gripperActionsReady )
		    {
			    cout << "gripperActionReady = " << gripperActionsReady << endl;
		    }	
		    gripperActionsReady = false;
		    sleep(1);

            cout << "press to continue" << endl;
            cin >> a;

           // move robot to the top of the lid on x and y axis,
		    cout << "robot action: move up from release position of objects" << endl;
            sleep(0.5);
		    robot_actions_msg.action = "moveToCartesian";
		    robot_actions_msg.position = 0;
		    robot_actions_msg.speed = 0;
		    robot_actions_msg.force = 0;
		    robot_actions_msg.forceDetection = false;
    		robot_actions_msg.incrementXaxis = 0.0;
    		robot_actions_msg.incrementYaxis = 0.0;
		    robot_actions_msg.incrementZaxis = 0.090;
		    robot_actions_pub.publish(robot_actions_msg);
		    while( !robotActionsReady )
		    {
			    cout << "robotActionReady = " << robotActionsReady << endl;
		    }	
		    robotActionsReady = false;

            cout << "press to continue" << endl;
            cin >> a;

		    cout << "robot action: moveToHomePosition" << endl;
            sleep(0.5);
		    robot_actions_msg.action = "moveToHomePosition";
		    robot_actions_pub.publish(robot_actions_msg);
		    while( !robotActionsReady )
		    {
			    cout << "robotActionReady = " << robotActionsReady << endl;
		    }	
		    robotActionsReady = false;

        // End of task 3: remove coin battery
        /****************************************************/
       }
        else if( taskNumber == 4 )
        {
            /****************************************************/
        // Beginning of task 4: remove ethernet cable
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

            // move robot 
		    cout << "robot action: move to release position of objects" << endl;
            sleep(0.5);
		    robot_actions_msg.action = "moveToCartesian";
		    robot_actions_msg.position = 0;
		    robot_actions_msg.speed = 0;
		    robot_actions_msg.force = 0;
		    robot_actions_msg.forceDetection = false;
    		robot_actions_msg.incrementXaxis = 0.010;
    		robot_actions_msg.incrementYaxis = 0.020;
		    robot_actions_msg.incrementZaxis = 0.0;
		    robot_actions_pub.publish(robot_actions_msg);
		    while( !robotActionsReady )
		    {
			    cout << "robotActionReady = " << robotActionsReady << endl;
		    }	
		    robotActionsReady = false;

            int a = 0;
            cout << "press to continue" << endl;            
            cin >> a;

            // rotate the robot's last joint to insert battery
		    cout << "robot action: rotate end effector to insert battery" << endl;
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
       		robot_actions_msg.robotJoints[5] = -90.0; // * signBattery;
		    robot_actions_pub.publish(robot_actions_msg);
		    while( !robotActionsReady )
		    {
			    cout << "robotActionReady = " << robotActionsReady << endl;
		    }	
		    robotActionsReady = false;

        // open gripper to sepcific distance to extract batteries
		    cout << "gripper action: open gripper to release battery" << endl;
		    gripper_actions_msg.action = "open";
		    gripper_actions_msg.position = 170;
		    gripper_actions_msg.speed = 50;
		    gripper_actions_msg.force = 200;
		    gripper_actions_msg.useContactDetection = false;
		    gripper_actions_pub.publish(gripper_actions_msg);
		    while( !gripperActionsReady )
		    {
			    cout << "gripperActionReady = " << gripperActionsReady << endl;
		    }	
		    gripperActionsReady = false;
		    sleep(1);
            
            cout << "press to continue" << endl;            
            cin >> a;

            geometry_msgs::Pose homePosition = move_group.getCurrentPose().pose;

            float ethernetCableX = 0.31035;
            float ethernetCableY = -0.01744;

        // move robot to the top of the lid on x and y axis,
		    cout << "robot action: move to x and y to top of coin battery" << endl;
            sleep(0.5);
		    robot_actions_msg.action = "moveToCartesian";
		    robot_actions_msg.position = 0;
		    robot_actions_msg.speed = 0;
		    robot_actions_msg.force = 0;
		    robot_actions_msg.forceDetection = false;
		    robot_actions_msg.incrementXaxis = ethernetCableX - homePosition.position.x;
		    robot_actions_msg.incrementYaxis = ethernetCableY - homePosition.position.y;
		    robot_actions_msg.incrementZaxis = 0.0;
		    robot_actions_pub.publish(robot_actions_msg);
		    while( !robotActionsReady )
		    {
			    cout << "robotActionReady = " << robotActionsReady << endl;
		    }	
		    robotActionsReady = false;

            cout << "press to continue" << endl;            
            cin >> a;

        // move robot down
		    cout << "robot action: move to release position of objects" << endl;
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
            
            cout << "press to continue" << endl;            
            cin >> a;

            // move robot up
		    cout << "robot action: move to release position of objects" << endl;
            sleep(0.5);
		    robot_actions_msg.action = "moveToCartesian";
		    robot_actions_msg.position = 0;
		    robot_actions_msg.speed = 0;
		    robot_actions_msg.force = 0;
		    robot_actions_msg.forceDetection = false;
    		robot_actions_msg.incrementXaxis = 0.0;
    		robot_actions_msg.incrementYaxis = 0.0;
		    robot_actions_msg.incrementZaxis = 0.010;
		    robot_actions_pub.publish(robot_actions_msg);
		    while( !robotActionsReady )
		    {
			    cout << "robotActionReady = " << robotActionsReady << endl;
		    }	
		    robotActionsReady = false;

            cout << "press to continue" << endl;            
            cin >> a;
          
            // close gripper to sepcific distance to extract batteries
		    cout << "gripper action: open gripper to release battery" << endl;
		    gripper_actions_msg.action = "close";
		    gripper_actions_msg.position = 230;
		    gripper_actions_msg.speed = 50;
		    gripper_actions_msg.force = 200;
		    gripper_actions_msg.useContactDetection = true;
		    gripper_actions_pub.publish(gripper_actions_msg);
		    while( !gripperActionsReady )
		    {
			    cout << "gripperActionReady = " << gripperActionsReady << endl;
		    }	
		    gripperActionsReady = false;
            sleep(1.0);

            cout << "press to continue" << endl;            
            cin >> a;

            // move robot on x axis to remove the ethernet cable
		    cout << "robot action: move to release position of objects" << endl;
            sleep(0.5);
		    robot_actions_msg.action = "moveToCartesian";
		    robot_actions_msg.position = 0;
		    robot_actions_msg.speed = 0;
		    robot_actions_msg.force = 0;
		    robot_actions_msg.forceDetection = false;
    		robot_actions_msg.incrementXaxis = 0.003;
    		robot_actions_msg.incrementYaxis = 0.0;
		    robot_actions_msg.incrementZaxis = 0.0;
		    robot_actions_pub.publish(robot_actions_msg);
		    while( !robotActionsReady )
		    {
			    cout << "robotActionReady = " << robotActionsReady << endl;
		    }	
		    robotActionsReady = false;

            // move robot up
		    cout << "robot action: move to release position of objects" << endl;
            sleep(0.5);
		    robot_actions_msg.action = "moveToCartesian";
		    robot_actions_msg.position = 0;
		    robot_actions_msg.speed = 0;
		    robot_actions_msg.force = 0;
		    robot_actions_msg.forceDetection = false;
    		robot_actions_msg.incrementXaxis = 0.0;
    		robot_actions_msg.incrementYaxis = 0.0;
		    robot_actions_msg.incrementZaxis = 0.020;
		    robot_actions_pub.publish(robot_actions_msg);
		    while( !robotActionsReady )
		    {
			    cout << "robotActionReady = " << robotActionsReady << endl;
		    }	
		    robotActionsReady = false;
            cout << "press to continue" << endl;  
            cin >> a;
            
            // move robot on x axis to remove the ethernet cable
		    cout << "robot action: move to release position of objects" << endl;
            sleep(0.5);
		    robot_actions_msg.action = "moveToCartesian";
		    robot_actions_msg.position = 0;
		    robot_actions_msg.speed = 0;
		    robot_actions_msg.force = 0;
		    robot_actions_msg.forceDetection = false;
    		robot_actions_msg.incrementXaxis = -0.003;
    		robot_actions_msg.incrementYaxis = 0.040;
		    robot_actions_msg.incrementZaxis = 0.0;
		    robot_actions_pub.publish(robot_actions_msg);
		    while( !robotActionsReady )
		    {
			    cout << "robotActionReady = " << robotActionsReady << endl;
		    }	
		    robotActionsReady = false;             

            geometry_msgs::Pose EtnernetHolePos = move_group.getCurrentPose().pose;
      
            cout << "press to continue" << endl;  
            cin >> a;
            
            // move robot on x axis to remove the ethernet cable
		    cout << "robot action: move to release position of objects" << endl;
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

            cout << "press to continue" << endl;  
            cin >> a;

        bool etnernetReleasedReady = false;
        float EtnernetHolePosX = EtnernetHolePos.position.x;
        float EtnernetHolePosY = EtnernetHolePos.position.y;
        float incX = 0.0;
        float incY = 0.0;
     while( etnernetReleasedReady == false )
        {
            cout << "In while loop, press to continue" << endl;
            cin >>  a;

            geometry_msgs::Pose EtnernetPos = move_group.getCurrentPose().pose;

            EtnernetHolePosX = EtnernetHolePosX + incX;
            EtnernetHolePosY = EtnernetHolePosY + incY;

         // move robot on y axis to the battery hole
		    cout << "robot action: move robot to hole to insert battery" << endl;
            sleep(0.5);
		    robot_actions_msg.action = "moveToCartesian";
		    robot_actions_msg.position = 0;
		    robot_actions_msg.speed = 0;
		    robot_actions_msg.force = 0;
		    robot_actions_msg.forceDetection = false;
		    robot_actions_msg.incrementXaxis = EtnernetHolePosX - EtnernetPos.position.x;
		    robot_actions_msg.incrementYaxis = EtnernetHolePosY - EtnernetPos.position.y;
		    robot_actions_msg.incrementZaxis = 0.0;
		    robot_actions_pub.publish(robot_actions_msg);
		    while( !robotActionsReady )
		    {
			    cout << "robotActionReady = " << robotActionsReady << endl;
		    }	
		    robotActionsReady = false;  

            EtnernetPos = move_group.getCurrentPose().pose;
            cout << "Current x, y, z position: " << EtnernetPos.position.x << ", " << EtnernetPos.position.y << ", " << EtnernetPos.position.z << endl;
            cout << "Increment x, y position: " << incX << ", " << incX << endl;

            // move robot on z axis down to the battery hole
		    cout << "robot action: move robot down to insert battery" << endl;
            sleep(0.5);
		    robot_actions_msg.action = "moveToCartesian";
		    robot_actions_msg.position = 0;
		    robot_actions_msg.speed = 0;
		    robot_actions_msg.force = 0;
		    robot_actions_msg.forceDetection = true;
		    robot_actions_msg.incrementXaxis = 0.0;
		    robot_actions_msg.incrementYaxis = 0.0;
		    robot_actions_msg.incrementZaxis = -0.078;

            cout << "Target x, y, z position: " << EtnernetPos.position.x << ", " << EtnernetPos.position.y << ", " << EtnernetPos.position.z - 0.070 << endl;
            cout << "press to continue" << endl;
            cin >> a;


		    robot_actions_pub.publish(robot_actions_msg);
		    while( !robotActionsReady )
		    {
			    cout << "robotActionReady = " << robotActionsReady << endl;
		    }	
		    robotActionsReady = false;
            
            geometry_msgs::Pose releasePositionCheck = move_group.getCurrentPose().pose;

            cout << "Released position check: " << releasePositionCheck << endl;

            cout << "press to continue" << endl;
            cin >> a;

            if( releasePositionCheck.position.z <= 0.40){
                etnernetReleasedReady = true;
                ReleasePosEthernet = move_group.getCurrentPose().pose;
                }            
            else
                etnernetReleasedReady = false;


            cout << "Current batteryReleasedReady = " << etnernetReleasedReady << endl;
            cout << "press to continue" << endl;
            cin >> a;

            if( etnernetReleasedReady == false )
            {
                // move robot on z axis down to the battery hole
		        cout << "robot action: move robot up to insert battery" << endl;
                sleep(0.5);
		        robot_actions_msg.action = "moveToCartesian";
		        robot_actions_msg.position = 0;
		        robot_actions_msg.speed = 0;
		        robot_actions_msg.force = 0;
		        robot_actions_msg.forceDetection = false;
		        robot_actions_msg.incrementXaxis = 0.0;
		        robot_actions_msg.incrementYaxis = 0.0;
		        robot_actions_msg.incrementZaxis = 0.020;
		        robot_actions_pub.publish(robot_actions_msg);
		        while( !robotActionsReady )
		        {
			        cout << "robotActionReady = " << robotActionsReady << endl;
		        }	
		        robotActionsReady = false;

                cout << "Input increment in X: " << endl;
                cin >> incX;
                cout << "Input increment in Y: " << endl;
                cin >> incY;
            }
                
        }              

        }

        else if( taskNumber == 5 )
        {
             /****************************************************/
        // Beginning of task 5: kesy task
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

            float keyCenterPosX = 0.246;
            float keyCenterPosY = -0.0650; //-0.0665;
            geometry_msgs::Pose homePosition = move_group.getCurrentPose().pose;

            
            // move robot 
		    cout << "robot action: move to release position of objects" << endl;
            sleep(0.5);
		    robot_actions_msg.action = "moveToCartesian";
		    robot_actions_msg.position = 0;
		    robot_actions_msg.speed = 0;
		    robot_actions_msg.force = 0;
		    robot_actions_msg.forceDetection = false;
    		robot_actions_msg.incrementXaxis = keyCenterPosX - homePosition.position.x;
    		robot_actions_msg.incrementYaxis = keyCenterPosY - homePosition.position.y;
		    robot_actions_msg.incrementZaxis = 0.0;
		    robot_actions_pub.publish(robot_actions_msg);
		    while( !robotActionsReady )
		    {
			    cout << "robotActionReady = " << robotActionsReady << endl;
		    }	
		    robotActionsReady = false;

            int a = 0;
            cout << "press to continue" << endl;            
            cin >> a;

            // rotate the robot's last joint to insert battery
		    cout << "robot action: rotate end effector to insert battery" << endl;
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

        // open gripper to sepcific distance to extract batteries
		    cout << "gripper action: open gripper to release battery" << endl;
		    gripper_actions_msg.action = "open";
		    gripper_actions_msg.position = 170;
		    gripper_actions_msg.speed = 50;
		    gripper_actions_msg.force = 200;
		    gripper_actions_msg.useContactDetection = false;
		    gripper_actions_pub.publish(gripper_actions_msg);
		    while( !gripperActionsReady )
		    {
			    cout << "gripperActionReady = " << gripperActionsReady << endl;
		    }	
		    gripperActionsReady = false;
		    sleep(1);
            
            cout << "press to continue" << endl;            
            cin >> a;

            // move robot down
		    cout << "robot action: move to release position of objects" << endl;
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

            cout << "press to continue" << endl;            
            cin >> a;

            // move robot up
		    cout << "robot action: move to release position of objects" << endl;
            sleep(0.5);
		    robot_actions_msg.action = "moveToCartesian";
		    robot_actions_msg.position = 0;
		    robot_actions_msg.speed = 0;
		    robot_actions_msg.force = 0;
		    robot_actions_msg.forceDetection = false;
    		robot_actions_msg.incrementXaxis = 0.0;
    		robot_actions_msg.incrementYaxis = 0.0;
		    robot_actions_msg.incrementZaxis = 0.005;
		    robot_actions_pub.publish(robot_actions_msg);
		    while( !robotActionsReady )
		    {
			    cout << "robotActionReady = " << robotActionsReady << endl;
		    }	
		    robotActionsReady = false;

            cout << "press to continue" << endl;            
            cin >> a;

            // open gripper to sepcific distance to extract batteries
		    cout << "gripper action: open gripper to release battery" << endl;
		    gripper_actions_msg.action = "open";
		    gripper_actions_msg.position = 240;
		    gripper_actions_msg.speed = 50;
		    gripper_actions_msg.force = 220;
		    gripper_actions_msg.useContactDetection = true;
		    gripper_actions_pub.publish(gripper_actions_msg);
		    while( !gripperActionsReady )
		    {
			    cout << "gripperActionReady = " << gripperActionsReady << endl;
		    }	
		    gripperActionsReady = false;
		    sleep(1);

            cout << "press to continue" << endl;            
            cin >> a;

            // move robot up
		    cout << "robot action: move to release position of objects" << endl;
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

            // rotate the robot's last joint to insert battery
		    cout << "robot action: rotate end effector to insert battery" << endl;
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

           cout << "press to continue" << endl;            
            cin >> a;

            geometry_msgs::Pose keytoLock = move_group.getCurrentPose().pose;
            float lockCenterPosX = 0.3106;
            float lockCenterPosY = -0.062;    
            // move robot 
		    cout << "robot action: move to release position of objects" << endl;
            sleep(0.5);
		    robot_actions_msg.action = "moveToCartesian";
		    robot_actions_msg.position = 0;
		    robot_actions_msg.speed = 0;
		    robot_actions_msg.force = 0;
		    robot_actions_msg.forceDetection = false;
    		robot_actions_msg.incrementXaxis = lockCenterPosX - keytoLock.position.x;
    		robot_actions_msg.incrementYaxis = lockCenterPosY - keytoLock.position.y;
		    robot_actions_msg.incrementZaxis = 0.0;
		    robot_actions_pub.publish(robot_actions_msg);
		    while( !robotActionsReady )
		    {
			    cout << "robotActionReady = " << robotActionsReady << endl;
		    }	
		    robotActionsReady = false;

           cout << "press to continue" << endl;            
            cin >> a;

        }

        else if( taskNumber == 6 )
        {
            break;
        }
        else
        {
            cout << "Input correct number" << endl;
        }
	}

    usleep(1000);
    ros::waitForShutdown();

    return 0;
}
