/*
 *********************************************************************************
 * Author: Uriel Martinez-Hernandez
 * Email: u.martinez@bath.ac.uk
 * Date: 19-December-2020
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

//#include <ur_controllers/speed_scaling_state_controller.h>
#include "ur_msgs/SetSpeedSliderFraction.h" //type of the service for speed slider

//#include <ur_robot_driver/hardware_interface.h>
#include <pluginlib/class_list_macros.hpp>
#include <ur_client_library/ur/tool_communication.h>
#include <ur_client_library/exceptions.h>

using namespace std;

double currentForceX = 0.0;
double currentForceY = 0.0;
double currentForceZ = 0.0;
double maxForceX = -1.0;
double maxForceY = -1.0;
double maxForceZ = -1.0;
float velocity_constant = 0.0;
bool contactDetect = false;
double globalUpdatedMaxForceZ = 0.0;
bool contactZ = false;
bool contactPressure = false;
double currentPressure = 0.0;



int main(int argc, char** argv)
{
    ros::init(argc, argv, "stopRobotTest");

    ros::NodeHandle node_handle;

    ros::Rate loop_rate(10000);
    ros::AsyncSpinner spinner(1);
    spinner.start();

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
    visual_tools.publishText(text_pose, "TACTO exploration - v 0.1.0", rvt::WHITE, rvt::XLARGE);
    visual_tools.trigger();


    ROS_INFO_NAMED("UR3 robot", "Reference frame: %s", move_group.getPlanningFrame().c_str());
    ROS_INFO_NAMED("UR3 robot", "End effector link: %s", move_group.getEndEffectorLink().c_str());
    ROS_INFO_NAMED("UR3 robot", "Available Planning Groups:");
    std::copy(move_group.getJointModelGroupNames().begin(), move_group.getJointModelGroupNames().end(), std::ostream_iterator<std::string>(std::cout, ", "));

    moveit::planning_interface::MoveGroupInterface::Plan plan;
    moveit::core::RobotStatePtr current_state = move_group.getCurrentState();
    std::vector<double> joint_group_positions;
    current_state->copyJointGroupPositions(joint_model_group, joint_group_positions);

/*
    double PLAN_TIME_ = 10.0;
    bool MOVEIT_REPLAN_ = true;

    move_group.setPlanningTime(PLAN_TIME_);
    move_group.allowReplanning(MOVEIT_REPLAN_);
*/
    move_group.setMaxVelocityScalingFactor(0.0);
    move_group.setMaxAccelerationScalingFactor(0.0);



    // move robot to home position
    double shoulder_pan_value = -23.64;
    double shoulder_lift_value = -84.94;
    double elbow_value = 45.46;
    double wrist_1_value = -49.61;
    double wrist_2_value = -90.08;
    double wrist_3_value = 334.00;
    bool success = false;

    // start moving robot to home position
    std::map<std::string, double> homePosition;

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
    sleep(2);

    move_group.setStartState(*move_group.getCurrentState());
    geometry_msgs::Pose homePositionPose = move_group.getCurrentPose().pose;


    // prepare z steps for sensor exploration
    float incrementZaxis = 0.080;

    move_group.setStartState(*move_group.getCurrentState());
    std::vector<geometry_msgs::Pose> waypoints;
    geometry_msgs::Pose pressButtonPosition = homePositionPose;
    pressButtonPosition.position.z = pressButtonPosition.position.z - incrementZaxis;
    waypoints.push_back(pressButtonPosition);

    // We want the Cartesian path to be interpolated at a resolution of 1 cm
    // which is why we will specify 0.01 as the max step in Cartesian
    // translation.  We will specify the jump threshold as 0.0, effectively disabling it.
    // Warning - disabling the jump threshold while operating real hardware can cause
    // large unpredictable motions of redundant joints and could be a safety issue
    moveit_msgs::RobotTrajectory trajectory;
    const double jump_threshold = 0.0;
    const double eef_step = 0.01;
    double fraction = move_group.computeCartesianPath(waypoints, eef_step, jump_threshold, trajectory, true);
    ROS_INFO_NAMED("Robothon", "Visualizing plan - press blue button (Cartesian path) (%.2f%% acheived)", fraction * 100.0);

    move_group.asyncExecute(trajectory);

//    sleep(2.5);
    int a = 0;
    cout << "press a key: ";
    cin >> a;

    move_group.stop();

    move_group.setStartState(*move_group.getCurrentState());
    std::vector<geometry_msgs::Pose> waypoints2;
    geometry_msgs::Pose secondButtonPosition = move_group.getCurrentPose().pose;
    waypoints2.push_back(secondButtonPosition);

    moveit_msgs::RobotTrajectory trajectory2;
    fraction = move_group.computeCartesianPath(waypoints2, eef_step, jump_threshold, trajectory2, true);
    ROS_INFO_NAMED("Robothon", "Visualizing plan - moving back to home position (Cartesian path) (%.2f%% acheived)", fraction * 100.0);

    move_group.execute(trajectory2);

/*
    sleep(2.0);


    move_group.setStartState(*move_group.getCurrentState());
    std::vector<geometry_msgs::Pose> waypoints2;
    geometry_msgs::Pose secondButtonPosition = move_group.getCurrentPose().pose;
    waypoints2.push_back(secondButtonPosition);
    secondButtonPosition.position.z = secondButtonPosition.position.z + incrementZaxis;
    waypoints2.push_back(secondButtonPosition);

    moveit_msgs::RobotTrajectory trajectory2;
    fraction = move_group.computeCartesianPath(waypoints2, eef_step, jump_threshold, trajectory2, true);
    ROS_INFO_NAMED("Robothon", "Visualizing plan - moving back to home position (Cartesian path) (%.2f%% acheived)", fraction * 100.0);

    move_group.execute(trajectory2);
*/
    return 0;

}
