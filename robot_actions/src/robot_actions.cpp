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


using namespace std;

double currentForceX = 0.0;
double currentForceY = 0.0;
double currentForceZ = 0.0;

float robotJoints[6];

robot_actions::robotControlParameters robot_action_mode;

// callback function to control the gripper action
void robotControlParametersCallback(const robot_actions::robotControlParameters::ConstPtr& msg)
{
    robot_action_mode.action = msg->action;
    robot_action_mode.position = msg->position;
    robot_action_mode.speed = msg->speed;
    robot_action_mode.forceDetection = msg->forceDetection;
    robot_action_mode.incrementXaxis = msg->incrementXaxis;
    robot_action_mode.incrementYaxis = msg->incrementYaxis;
    robot_action_mode.incrementZaxis = msg->incrementZaxis;
	robot_action_mode.robotJoints = msg->robotJoints;
}

void ftSensorCallback(const robotiq_ft_sensor::ft_sensor& msg)
{
    currentForceX = msg.Fx;
    currentForceY = msg.Fy;
    currentForceZ = msg.Fz;
}


int main(int argc, char** argv)
{
    ros::init(argc, argv, "robot_actions");

    ros::NodeHandle node_handle;

    ros::Rate loop_rate(10);
    ros::AsyncSpinner spinner(1);
    spinner.start();

    std_msgs::Bool msgRobotActionStatus;

    // connection of publisher and subscriber with the Robotiq controller from ROS Industrial
    ros::Subscriber robotControlParametersSub = node_handle.subscribe<robot_actions::robotControlParameters>("robot_actions", 1, robotControlParametersCallback);
    ros::Publisher robotActionStatusPub = node_handle.advertise<std_msgs::Bool>("robot_actions_status", 1000);

    ros::Subscriber RobotiqFTSensor = node_handle.subscribe("robotiq_ft_sensor", 1000, ftSensorCallback);
    ros::ServiceClient clientFTSensor = node_handle.serviceClient<robotiq_ft_sensor::sensor_accessor>("robotiq_ft_sensor_acc");

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

    double PLAN_TIME_ = 100.0;
    bool MOVEIT_REPLAN_ = true;

    move_group.setPlanningTime(PLAN_TIME_);
    move_group.allowReplanning(MOVEIT_REPLAN_);

    move_group.setMaxVelocityScalingFactor(0.50);
    move_group.setMaxAccelerationScalingFactor(0.50);

    ur_msgs::SetSpeedSliderFraction set_speed_frac;
    set_speed_frac.request.speed_slider_fraction = 0.20; // change velocity in the slider of teach pendant
    clientSpeedSlider.call(set_speed_frac);   

    // start moving robot to home position
    std::map<std::string, double> homePosition;

    geometry_msgs::Pose homePositionPose;
    bool setHomePositionReady = false;
    std::vector<geometry_msgs::Pose> waypoints;

    moveit_msgs::RobotTrajectory trajectory;
    const double jump_threshold = 0.0;
    const double eef_step = 0.01;
    double fraction;


    while( ros::ok() )
    {
        cout << "Waiting for actions..." << endl;
        sleep(1);
        if( robot_action_mode.action == "moveToHomePosition" )    // define the home position needed to enable the rest of actions
        {
            move_group.setStartState(*move_group.getCurrentState());

            // move robot to home position
            double shoulder_pan_value = -23.64;
            double shoulder_lift_value = -84.94;
            double elbow_value = 45.46;
            double wrist_1_value = -49.61;
            double wrist_2_value = -90.08;
            double wrist_3_value = 334.00;
            bool success = false;

            homePosition["shoulder_pan_joint"] = shoulder_pan_value * 3.1416 / 180;	// (deg*PI/180)
            homePosition["shoulder_lift_joint"] = shoulder_lift_value * 3.1416 / 180;
            homePosition["elbow_joint"] = elbow_value * 3.1416 / 180;
            homePosition["wrist_1_joint"] = wrist_1_value * 3.1416 / 180;
            homePosition["wrist_2_joint"] = wrist_2_value * 3.1416 / 180;
            homePosition["wrist_3_joint"] = wrist_3_value * 3.1416 / 180;

            move_group.setJointValueTarget(homePosition);

            success = (move_group.plan(plan) == moveit::planning_interface::MoveItErrorCode::SUCCESS);
            ROS_INFO("Visualizing PRE-START position plan (%.2f%% acheived)",success * 100.0);

            move_group.execute(plan);

            if( ( success * 100.0 ) == 100.0 )
                setHomePositionReady = true;
            else
                setHomePositionReady = false;            

            move_group.setStartState(*move_group.getCurrentState());
            homePositionPose = move_group.getCurrentPose().pose;

            msgRobotActionStatus.data = true;
            robotActionStatusPub.publish(msgRobotActionStatus);
            sleep(0.5);

            robot_action_mode.action = "empty";
            robot_action_mode.position = 0;
            robot_action_mode.speed = 0;
            robot_action_mode.forceDetection = 0;
            robot_action_mode.incrementXaxis = 0;
            robot_action_mode.incrementYaxis = 0;
            robot_action_mode.incrementZaxis = 0;
			robot_action_mode.robotJoints[0] = 0.0;
			robot_action_mode.robotJoints[1] = 0.0;
			robot_action_mode.robotJoints[2] = 0.0;
			robot_action_mode.robotJoints[3] = 0.0;
			robot_action_mode.robotJoints[4] = 0.0;
			robot_action_mode.robotJoints[5] = 0.0;
/*			robot_action_mode.shoulder_pan_value = 0.0;
			robot_action_mode.shoulder_lift_value = 0.0;
			robot_action_mode.elbow_value = 0.0;
			robot_action_mode.wrist_1_value = 0.0;
			robot_action_mode.wrist_2_value = 0.0;
			robot_action_mode.wrist_3_value = 0.0;
*/
            sleep(2);
        }
        else if( robot_action_mode.action == "moveDown" )    // move the robot down
        {
            if( setHomePositionReady == true )
            {
                // prepare z steps for sensor exploration
                float incrementZaxis = 0.078;
                
                if( robot_action_mode.forceDetection == false )
                {
                    incrementZaxis = robot_action_mode.incrementZaxis;
                }

                move_group.setStartState(*move_group.getCurrentState());
                geometry_msgs::Pose currentPose = move_group.getCurrentPose().pose;
                geometry_msgs::Pose targetPose = move_group.getCurrentPose().pose;
                targetPose.position.z = targetPose.position.z - incrementZaxis;
                waypoints.clear();
                waypoints.push_back(targetPose);

                cout << "Current z position: " << currentPose.position.z << endl;
                cout << "Increment z position: " << incrementZaxis << endl;
                cout << "Target z position: " << targetPose.position.z << endl;

                fraction = move_group.computeCartesianPath(waypoints, eef_step, jump_threshold, trajectory, true);
                ROS_INFO_NAMED("Robothon", "Visualizing plan (Cartesian path) (%.2f%% acheived)", fraction * 100.0);

                // Visualize the plan in RViz
                visual_tools.deleteAllMarkers();
                visual_tools.publishText(text_pose, "Cartesian Path", rvt::WHITE, rvt::XLARGE);
                visual_tools.publishPath(waypoints, rvt::LIME_GREEN, rvt::SMALL);
        
                for (std::size_t i = 0; i < waypoints.size(); ++i)
                    visual_tools.publishAxisLabeled(waypoints[i], "pt" + std::to_string(i), rvt::SMALL);

                visual_tools.trigger();

                set_speed_frac.request.speed_slider_fraction = 0.10; // change velocity
                clientSpeedSlider.call(set_speed_frac);   

                if( robot_action_mode.forceDetection == false )
                {
                    move_group.execute(trajectory);
                }
                else
                {
                    bool maxThresholdZ = false;
                    bool contactZ = false;
                    bool contactPressure = false;

                    double updatedMaxForceX = 0.0;
                    double updatedMaxForceY = 0.0;
                    double updatedMaxForceZ = 0.0;

                    double avgFx = 0.0;
                    double avgFy = 0.0;
                    double avgFz = 0.0;

                    double maxForceX = -1.0;
                    double maxForceY = -1.0;
                    double maxForceZ = -1.0;


                    int numberOfForceSamples = 50;

                    while( maxThresholdZ == false )
                    {
                        avgFx = 0.0;
                        avgFy = 0.0;
                        avgFz = 0.0;

                        srv.request.command_id = srv.request.COMMAND_SET_ZERO; // set force and torque values of the FT sensor 0 
                        if( clientFTSensor.call( srv ) )
                        {
                            ROS_INFO("ret: %s", srv.response.res.c_str());
                        }



                        for(int i = 0; i < numberOfForceSamples; i++)
                        {
                            avgFx = avgFx + currentForceX;
                            avgFy = avgFy + currentForceY;
                            avgFz = avgFz + currentForceZ;
                            sleep(0.1);
                        }

                        avgFx = (avgFx / numberOfForceSamples);
                        avgFy = (avgFy / numberOfForceSamples);
                        avgFz = (avgFz / numberOfForceSamples);

                        updatedMaxForceX = maxForceX + avgFx;
                        updatedMaxForceY = maxForceY + avgFy;
                        updatedMaxForceZ = avgFz + (avgFz * 10.00);

                        if( fabs(updatedMaxForceZ) >= 1.0 && fabs(updatedMaxForceZ) <= 3.0 )
                            maxThresholdZ = true;
                        else
                            cout << "Adjusting maximum threshold in Z axis" << endl;
                    }

                    if( updatedMaxForceZ > 0 )
                        updatedMaxForceZ = -1 * updatedMaxForceZ;

                    cout << "avgFx = " << avgFx << endl;
                    cout << "avgFy = " << avgFy << endl;
                    cout << "avgFx = " << avgFz << endl;

                    cout << "updatedMaxForceX = " << updatedMaxForceX << endl;
                    cout << "updatedMaxForceY = " << updatedMaxForceY << endl;
                    cout << "updatedMaxForceZ = " << updatedMaxForceZ << endl;

                    sleep(0.5);

                    srv.request.command_id = srv.request.COMMAND_SET_ZERO; // set force and torque values of the FT sensor 0 
                    if( clientFTSensor.call( srv ) )
                    {
                        ROS_INFO("ret: %s", srv.response.res.c_str());
                    }

                    move_group.asyncExecute(trajectory);

                    while( ( contactZ == false ) && ( currentPose.position.z > targetPose.position.z ) )
                    {
                        if( currentForceZ > 0 )
                            currentForceZ = -1 * currentForceZ;

                        geometry_msgs::Pose preStopPosition = move_group.getCurrentPose().pose;

                        if( currentForceZ <= updatedMaxForceZ )
                        {
                            move_group.stop();

                            contactZ = true;

                            move_group.setStartState(*move_group.getCurrentState());
                            geometry_msgs::Pose stopPosition = move_group.getCurrentPose().pose;
                            waypoints.clear();
                            waypoints.push_back(stopPosition);
                            preStopPosition.position.z = preStopPosition.position.z + 0.002;
                            waypoints.push_back(preStopPosition);

                            fraction = move_group.computeCartesianPath(waypoints, eef_step, jump_threshold, trajectory, true);
                            ROS_INFO_NAMED("Robothon", "Visualizing plan (Cartesian path) (%.2f%% acheived)", fraction * 100.0);

                            move_group.execute(trajectory);
    //                        move_group.asyncExecute(trajectory);

                            sleep(1.0);
                            cout << "==================================================" << endl;
                            cout << "updatedMaxForceZ = " << updatedMaxForceZ << endl;
                            cout << "maximum contact force detected in Z = " << currentForceZ << endl;
                            cout << "==================================================" << endl;
                        }

                        currentPose = move_group.getCurrentPose().pose;
                    }
                }

                set_speed_frac.request.speed_slider_fraction = 0.80; // change velocity
                clientSpeedSlider.call(set_speed_frac);   

				msgRobotActionStatus.data = true;
                robotActionStatusPub.publish(msgRobotActionStatus);
                sleep(0.5);

            }
            else
            {
				msgRobotActionStatus.data = false;
                robotActionStatusPub.publish(msgRobotActionStatus);
                sleep(0.5);
                cout << "Home position has not been set" << endl;
            }

            robot_action_mode.action = "empty";
            robot_action_mode.position = 0;
            robot_action_mode.speed = 0;
            robot_action_mode.forceDetection = 0;
            robot_action_mode.incrementXaxis = 0;
            robot_action_mode.incrementYaxis = 0;
            robot_action_mode.incrementZaxis = 0;
			robot_action_mode.robotJoints[0] = 0.0;
			robot_action_mode.robotJoints[1] = 0.0;
			robot_action_mode.robotJoints[2] = 0.0;
			robot_action_mode.robotJoints[3] = 0.0;
			robot_action_mode.robotJoints[4] = 0.0;
			robot_action_mode.robotJoints[5] = 0.0;
        }
        else if( robot_action_mode.action == "moveToCartesian" )    // move the robot to specific x, y, z position
        {
            if( setHomePositionReady == true )
            {
                // prepare z steps for sensor exploration
				float incrementXaxis = 0.0;
				float incrementYaxis = 0.0;
                float incrementZaxis = 0.0;
                
//                if( robot_action_mode.forceDetection == false )
//                {
//                    incrementZaxis = robot_action_mode.incrementZaxis;
//                }

                incrementXaxis = robot_action_mode.incrementXaxis;
                incrementYaxis = robot_action_mode.incrementYaxis;
                incrementZaxis = robot_action_mode.incrementZaxis;

                move_group.setStartState(*move_group.getCurrentState());
                geometry_msgs::Pose currentPose = move_group.getCurrentPose().pose;
                geometry_msgs::Pose targetPose = move_group.getCurrentPose().pose;

                targetPose.position.x = targetPose.position.x + incrementXaxis;
                targetPose.position.y = targetPose.position.y + incrementYaxis;
                targetPose.position.z = targetPose.position.z + incrementZaxis;
                waypoints.clear();
                waypoints.push_back(targetPose);

                cout << "Current x, y, z position: " << currentPose.position.x << ", " << currentPose.position.y << ", " << currentPose.position.z << endl;
                cout << "Increment x, y, z position: " << incrementXaxis << ", " << incrementYaxis << ", " << incrementZaxis << endl;
                cout << "Target x, y, z position: " << targetPose.position.x << ", " << targetPose.position.y << ", " << targetPose.position.z << endl;

                fraction = move_group.computeCartesianPath(waypoints, eef_step, jump_threshold, trajectory, true);
                ROS_INFO_NAMED("Robothon", "Visualizing plan (Cartesian path) (%.2f%% acheived)", fraction * 100.0);

                // Visualize the plan in RViz
                visual_tools.deleteAllMarkers();
                visual_tools.publishText(text_pose, "Cartesian Path", rvt::WHITE, rvt::XLARGE);
                visual_tools.publishPath(waypoints, rvt::LIME_GREEN, rvt::SMALL);
        
                for (std::size_t i = 0; i < waypoints.size(); ++i)
                    visual_tools.publishAxisLabeled(waypoints[i], "pt" + std::to_string(i), rvt::SMALL);

                visual_tools.trigger();

                set_speed_frac.request.speed_slider_fraction = 0.10; // change velocity
                clientSpeedSlider.call(set_speed_frac);   

                if( robot_action_mode.forceDetection == false )
                {
                    move_group.execute(trajectory);
                }
                else
                {
                    bool maxThresholdZ = false;
                    bool contactZ = false;
                    bool contactPressure = false;

                    double updatedMaxForceX = 0.0;
                    double updatedMaxForceY = 0.0;
                    double updatedMaxForceZ = 0.0;

                    double avgFx = 0.0;
                    double avgFy = 0.0;
                    double avgFz = 0.0;

                    double maxForceX = -1.0;
                    double maxForceY = -1.0;
                    double maxForceZ = -1.0;


                    int numberOfForceSamples = 50;

                    while( maxThresholdZ == false )
                    {
                        avgFx = 0.0;
                        avgFy = 0.0;
                        avgFz = 0.0;

                        srv.request.command_id = srv.request.COMMAND_SET_ZERO; // set force and torque values of the FT sensor 0 
                        if( clientFTSensor.call( srv ) )
                        {
                            ROS_INFO("ret: %s", srv.response.res.c_str());
                        }



                        for(int i = 0; i < numberOfForceSamples; i++)
                        {
                            avgFx = avgFx + currentForceX;
                            avgFy = avgFy + currentForceY;
                            avgFz = avgFz + currentForceZ;
                            sleep(0.1);
                        }

                        avgFx = (avgFx / numberOfForceSamples);
                        avgFy = (avgFy / numberOfForceSamples);
                        avgFz = (avgFz / numberOfForceSamples);

                        updatedMaxForceX = maxForceX + avgFx;
                        updatedMaxForceY = maxForceY + avgFy;
                        updatedMaxForceZ = avgFz + (avgFz * 10.00);

                        if( fabs(updatedMaxForceZ) >= 1.0 && fabs(updatedMaxForceZ) <= 3.0 )
                            maxThresholdZ = true;
                        else
                            cout << "Adjusting maximum threshold in Z axis" << endl;
                    }

                    if( updatedMaxForceZ > 0 )
                        updatedMaxForceZ = -1 * updatedMaxForceZ;

                    cout << "avgFx = " << avgFx << endl;
                    cout << "avgFy = " << avgFy << endl;
                    cout << "avgFx = " << avgFz << endl;

                    cout << "updatedMaxForceX = " << updatedMaxForceX << endl;
                    cout << "updatedMaxForceY = " << updatedMaxForceY << endl;
                    cout << "updatedMaxForceZ = " << updatedMaxForceZ << endl;

                    sleep(0.5);

                    srv.request.command_id = srv.request.COMMAND_SET_ZERO; // set force and torque values of the FT sensor 0 
                    if( clientFTSensor.call( srv ) )
                    {
                        ROS_INFO("ret: %s", srv.response.res.c_str());
                    }

                    move_group.asyncExecute(trajectory);

                    while( ( contactZ == false ) && ( currentPose.position.z > targetPose.position.z ) )
                    {
                        if( currentForceZ > 0 )
                            currentForceZ = -1 * currentForceZ;

                        geometry_msgs::Pose preStopPosition = move_group.getCurrentPose().pose;

                        if( currentForceZ <= updatedMaxForceZ )
                        {
                            move_group.stop();

                            contactZ = true;

                            move_group.setStartState(*move_group.getCurrentState());
                            geometry_msgs::Pose stopPosition = move_group.getCurrentPose().pose;
                            waypoints.clear();
                            waypoints.push_back(stopPosition);
                            preStopPosition.position.z = preStopPosition.position.z + 0.002;
                            waypoints.push_back(preStopPosition);

                            fraction = move_group.computeCartesianPath(waypoints, eef_step, jump_threshold, trajectory, true);
                            ROS_INFO_NAMED("Robothon", "Visualizing plan (Cartesian path) (%.2f%% acheived)", fraction * 100.0);

                            move_group.execute(trajectory);
    //                        move_group.asyncExecute(trajectory);

                            sleep(1.0);
                            cout << "==================================================" << endl;
                            cout << "updatedMaxForceZ = " << updatedMaxForceZ << endl;
                            cout << "maximum contact force detected in Z = " << currentForceZ << endl;
                            cout << "==================================================" << endl;
                        }

                        currentPose = move_group.getCurrentPose().pose;
                    }
                }

                currentPose = move_group.getCurrentPose().pose;
                cout << "Target x, y, z position: " << currentPose.position.x << ", " << currentPose.position.y << ", " << currentPose.position.z << endl;

                set_speed_frac.request.speed_slider_fraction = 0.80; // change velocity
                clientSpeedSlider.call(set_speed_frac);   

				msgRobotActionStatus.data = true;
                robotActionStatusPub.publish(msgRobotActionStatus);
                sleep(0.5);
            }
            else
            {
//				msgRobotActionStatus.data = false;
//                robotActionStatusPub.publish(msgRobotActionStatus);
//                sleep(0.5);
                cout << "Home position has not been set" << endl;
            }

            robot_action_mode.action = "empty";
            robot_action_mode.position = 0;
            robot_action_mode.speed = 0;
            robot_action_mode.forceDetection = 0;
            robot_action_mode.incrementXaxis = 0;
            robot_action_mode.incrementYaxis = 0;
            robot_action_mode.incrementZaxis = 0;
			robot_action_mode.robotJoints[0] = 0.0;
			robot_action_mode.robotJoints[1] = 0.0;
			robot_action_mode.robotJoints[2] = 0.0;
			robot_action_mode.robotJoints[3] = 0.0;
			robot_action_mode.robotJoints[4] = 0.0;
			robot_action_mode.robotJoints[5] = 0.0;
        }
        else if( robot_action_mode.action == "moveToPose" )    // move robot to specific pose
        {
            msgRobotActionStatus.data = false;

            robot_action_mode.action = "empty";
            robot_action_mode.position = 0;
            robot_action_mode.speed = 0;
            robot_action_mode.forceDetection = 0;
            robot_action_mode.incrementXaxis = 0;
            robot_action_mode.incrementYaxis = 0;
            robot_action_mode.incrementZaxis = 0;
			robot_action_mode.robotJoints[0] = 0.0;
			robot_action_mode.robotJoints[1] = 0.0;
			robot_action_mode.robotJoints[2] = 0.0;
			robot_action_mode.robotJoints[3] = 0.0;
			robot_action_mode.robotJoints[4] = 0.0;
			robot_action_mode.robotJoints[5] = 0.0;

        }
        else if( robot_action_mode.action == "moveToJoints" )    // move robot to specific joint configuration
        {
            if( setHomePositionReady == true )
            {
				move_group.setStartState(*move_group.getCurrentState());

				// start moving robot to home position
				std::map<std::string, double> jointPositions;
				bool success = false;

                current_state = move_group.getCurrentState();
                current_state->copyJointGroupPositions(joint_model_group, joint_group_positions);
                for( int i = 0; i < 6; i++ )
                    cout << "Current Joint value [" << i << "] = " << joint_group_positions[i] << ", Next Joint value [" << i << "] = " << robot_action_mode.robotJoints[i] << endl;


/*
				// move robot to home position
				double shoulder_pan_value = robot_action_mode.shoulder_pan_value;
				double shoulder_lift_value = robot_action_mode.shoulder_lift_value;
				double elbow_value = robot_action_mode.elbow_value;
				double wrist_1_value = robot_action_mode.wrist_1_value;
				double wrist_2_value = robot_action_mode.wrist_2_value;
				double wrist_3_value = robot_action_mode.wrist_3_value;
				bool success = false;

				jointPositions["shoulder_pan_joint"] = shoulder_pan_value * 3.1416 / 180;	// (deg*PI/180)
				jointPositions["shoulder_lift_joint"] = shoulder_lift_value * 3.1416 / 180;
				jointPositions["elbow_joint"] = elbow_value * 3.1416 / 180;
				jointPositions["wrist_1_joint"] = wrist_1_value * 3.1416 / 180;
				jointPositions["wrist_2_joint"] = wrist_2_value * 3.1416 / 180;
				jointPositions["wrist_3_joint"] = wrist_3_value * 3.1416 / 180;
*/
				jointPositions["shoulder_pan_joint"] = joint_group_positions[0] + (robot_action_mode.robotJoints[0] * 3.1416 / 180);	// (deg*PI/180)
				jointPositions["shoulder_lift_joint"] = joint_group_positions[1] + (robot_action_mode.robotJoints[1] * 3.1416 / 180);
				jointPositions["elbow_joint"] = joint_group_positions[2] + (robot_action_mode.robotJoints[2] * 3.1416 / 180);
				jointPositions["wrist_1_joint"] = joint_group_positions[3] + (robot_action_mode.robotJoints[3] * 3.1416 / 180);
				jointPositions["wrist_2_joint"] = joint_group_positions[4] + (robot_action_mode.robotJoints[4] * 3.1416 / 180);
				jointPositions["wrist_3_joint"] = joint_group_positions[5] + (robot_action_mode.robotJoints[5] * 3.1416 / 180);

				move_group.setJointValueTarget(jointPositions);

				success = (move_group.plan(plan) == moveit::planning_interface::MoveItErrorCode::SUCCESS);
				ROS_INFO("Visualizing PRE-START position plan (%.2f%% acheived)",success * 100.0);

				move_group.execute(plan);

				move_group.setStartState(*move_group.getCurrentState());
				homePositionPose = move_group.getCurrentPose().pose;
				msgRobotActionStatus.data = true;
                robotActionStatusPub.publish(msgRobotActionStatus);

                robot_action_mode.action = "empty";
                robot_action_mode.position = 0;
                robot_action_mode.speed = 0;
                robot_action_mode.forceDetection = 0;
                robot_action_mode.incrementXaxis = 0;
                robot_action_mode.incrementYaxis = 0;
                robot_action_mode.incrementZaxis = 0;
			    robot_action_mode.robotJoints[0] = 0.0;
			    robot_action_mode.robotJoints[1] = 0.0;
			    robot_action_mode.robotJoints[2] = 0.0;
			    robot_action_mode.robotJoints[3] = 0.0;
			    robot_action_mode.robotJoints[4] = 0.0;
			    robot_action_mode.robotJoints[5] = 0.0;
			}
            else
            {
				msgRobotActionStatus.data = false;
                cout << "Home position has not been set" << endl;
            }
			

            robot_action_mode.action = "empty";
            robot_action_mode.position = 0;
            robot_action_mode.speed = 0;
            robot_action_mode.forceDetection = 0;
            robot_action_mode.incrementXaxis = 0;
            robot_action_mode.incrementYaxis = 0;
            robot_action_mode.incrementZaxis = 0;
			robot_action_mode.robotJoints[0] = 0.0;
			robot_action_mode.robotJoints[1] = 0.0;
			robot_action_mode.robotJoints[2] = 0.0;
			robot_action_mode.robotJoints[3] = 0.0;
			robot_action_mode.robotJoints[4] = 0.0;
			robot_action_mode.robotJoints[5] = 0.0;
			
            sleep(2);
        }
        else
        {
            msgRobotActionStatus.data = false;
            robot_action_mode.action = "empty";
            cout << "No robot action found - check typing error - check robot action name" << endl;
        }    
    }

    usleep(1000);
    ros::waitForShutdown();

    return 0;
}
