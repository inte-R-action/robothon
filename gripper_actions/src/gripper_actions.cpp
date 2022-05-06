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

/*static const std::string PLANNING_GROUP = "manipulator";
moveit::planning_interface::MoveGroupInterface move_group(PLANNING_GROUP);
moveit::planning_interface::PlanningSceneInterface planning_scene_interface;
const robot_state::JointModelGroup* joint_model_group = move_group.getCurrentState()->getJointModelGroup(PLANNING_GROUP);
*/


// global variable to hold the status of the gripper
robotiq_2f_gripper_control::Robotiq2FGripper_robot_input gripperStatus;

robotiq_2f_gripper_control::Robotiq2FGripper_robot_output outputControlValues;

// global variable to hold the status of the tactile sensor
bool touchDetected = false;

// callback function to get the status signals from the gripper
void gripperStatusCallback(const robotiq_2f_gripper_control::Robotiq2FGripper_robot_input::ConstPtr& msg)
{
    gripperStatus = *msg;

}

void ftSensorCallback(const robotiq_ft_sensor::ft_sensor& msg)
{
//    int a = 0;
//    cout << msg.Fx << "\t" << msg.Fy << "\t" << msg.Fz << "\t" << msg.Mx<< "\t" << msg.My << "\t" << msg.Mz << "\n" << endl;
    currentForceX = msg.Fx;
    currentForceY = msg.Fy;
    currentForceZ = msg.Fz;

/*    if( currentForceZ <= globalUpdatedMaxForceZ )
    {
//        move_group.stop();
        contactZ = true;
        cout << "**************************************************" << endl;
        cout << "callback globalUpdatedMaxForceZ = " << globalUpdatedMaxForceZ << endl;
        cout << "callback maximum contact force detected in Z = " << currentForceZ << endl;
        cout << "**************************************************" << endl;
    }
*/
}

void pressureSensorCallback(const std_msgs::Float32::ConstPtr& msg)
{
    currentPressure = msg->data;
//    cout << "Pressure data: " << msg->data << endl;
}



int main(int argc, char** argv)
{
    ros::init(argc, argv, "gripper_actions");

    ros::NodeHandle node_handle;

    ros::Rate loop_rate(10000);
    ros::AsyncSpinner spinner(1);
    spinner.start();

    // connection of publisher and subscriber with the Robotiq controller from ROS Industrial
    ros::Publisher Robotiq2FGripperArgPub = node_handle.advertise<robotiq_2f_gripper_control::Robotiq2FGripper_robot_output>("Robotiq2FGripperRobotOutput", 1);
    ros::Subscriber Robotiq2FGripperStatusSub = node_handle.subscribe("Robotiq2FGripperRobotInput", 1, gripperStatusCallback);
    ros::Subscriber RobotiqFTSensor = node_handle.subscribe("robotiq_ft_sensor", 1000, ftSensorCallback);
    ros::ServiceClient clientFTSensor = node_handle.serviceClient<robotiq_ft_sensor::sensor_accessor>("robotiq_ft_sensor_acc");
  	ros::Subscriber subPressureData = node_handle.subscribe("pressureRosPort", 1000, pressureSensorCallback);

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



    double PLAN_TIME_ = 100.0;
    bool MOVEIT_REPLAN_ = true;

    move_group.setPlanningTime(PLAN_TIME_);
    move_group.allowReplanning(MOVEIT_REPLAN_);

    move_group.setMaxVelocityScalingFactor(0.50);
    move_group.setMaxAccelerationScalingFactor(0.50);




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

    ur_msgs::SetSpeedSliderFraction set_speed_frac;
    set_speed_frac.request.speed_slider_fraction = 0.20; // change velocity
    clientSpeedSlider.call(set_speed_frac);   


/*
    ros::Timer timer = node_handle.createTimer(ros::Duration(0.01), [&](const ros::TimerEvent&)
    {    
        cout << "current force z = " << currentForceZ << endl;
        if (abs(currentForceZ) >= abs(maxForceZ) )
        {
            contactDetect = true;
            cout << "Stop detected" << endl;
//            set_speed_frac.request.speed_slider_fraction = 0.05; // change velocity
//            clientSpeedSlider.call(set_speed_frac);
        }
        else
        {
//            set_speed_frac.request.speed_slider_fraction = 0.30; // change velocity
//            clientSpeedSlider.call(set_speed_frac);   
        }
    });
*/

/*    ros::Timer stopRobotTimer = node_handle.createTimer(ros::Duration(0.01), [&](const ros::TimerEvent&)
    {
        if( contactDetect == true )
            move_group.stop();
    });
*/


    while( ros::ok() )
    {
    printf("==================================================\n");
    // reset the robotic gripper (needed to activate the robot)
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


    // activate the robotic gripper
    outputControlValues.rACT = 1;
    outputControlValues.rGTO = 1;
    outputControlValues.rATR = 0;
    outputControlValues.rPR = 0;
    outputControlValues.rSP = 150;
    outputControlValues.rFR = 100;
    Robotiq2FGripperArgPub.publish(outputControlValues);
    std::cout << "ACTIVATE GRIPPER" << std::endl; 

    srv.request.command_id = srv.request.COMMAND_SET_ZERO; // set force and torque values of the FT sensor 0 
    if( clientFTSensor.call( srv ) )
    {
        ROS_INFO("ret: %s", srv.response.res.c_str());
    }

    sleep(2);

    // wait until the activation action is completed to continue with the next action
    while( gripperStatus.gSTA != 3 )
    {
//        printf("IN PROGRESS: gSTA [%d]\n", gripperStatus.gSTA);
//        usleep(1000);
    }

    printf("COMPLETED: gSTA [%d]\n", gripperStatus.gSTA);
    sleep(2);

    // set gripper to standby to clear the flags
    outputControlValues.rGTO = 0;
    Robotiq2FGripperArgPub.publish(outputControlValues);
    std::cout << "STANDBY GRIPPER" << std::endl; 
    sleep(2);


    // close the gripper to the maximum value of rPR = 255
    // rGTO = 1 allows the robot to perform an action
    outputControlValues.rGTO = 1;
    outputControlValues.rSP = 100;
    outputControlValues.rPR = 225;
    outputControlValues.rFR = 200;
    Robotiq2FGripperArgPub.publish(outputControlValues);
    std::cout << "CLOSE GRIPPER" << std::endl; 

    // wait until the activation action is completed to continue with the next action
//        while( gripperStatus.gOBJ != 3 && touchDetected == false )
    while( gripperStatus.gOBJ != 3 )
    {
//            printf("IN PROGRESS: gOBJ [%d]\n", gripperStatus.gOBJ);
//            usleep(1000);
    }

    printf("COMPLETED: gOBJ [%d]\n", gripperStatus.gOBJ);
    sleep(2);


    /*****************************************************************************************/
    // Start task for pressing blue button

    // prepare z steps for sensor exploration
    float incrementZaxis = 0.078;

    move_group.setStartState(*move_group.getCurrentState());
//    geometry_msgs::Pose homePositionPose = homePosition; //move_group.getCurrentPose().pose;

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

    // Visualize the plan in RViz
    visual_tools.deleteAllMarkers();
    visual_tools.publishText(text_pose, "Cartesian Path", rvt::WHITE, rvt::XLARGE);
    visual_tools.publishPath(waypoints, rvt::LIME_GREEN, rvt::SMALL);
    for (std::size_t i = 0; i < waypoints.size(); ++i)
    visual_tools.publishAxisLabeled(waypoints[i], "pt" + std::to_string(i), rvt::SMALL);
    visual_tools.trigger();
    // Cartesian motions should often be slow, e.g. when approaching objects. The speed of cartesian
    // plans cannot currently be set through the maxVelocityScalingFactor, but requires you to time
    // the trajectory manually, as described [here](https://groups.google.com/forum/#!topic/moveit-users/MOoFxy2exT4).
    // Pull requests are welcome.

    // You can execute a trajectory like this.
//    move_group.execute(trajectory);
//    geometry_msgs::Pose maxPositionZ = move_group.getCurrentPose().pose;

    srv.request.command_id = srv.request.COMMAND_SET_ZERO; // set force and torque values of the FT sensor 0 
    if( clientFTSensor.call( srv ) )
    {
        ROS_INFO("ret: %s", srv.response.res.c_str());
    }

    double avgFx = 0.0;
    double avgFy = 0.0;
    double avgFz = 0.0;

    for(int i = 0; i < 50; i++)
    {
        avgFx = avgFx + currentForceX;
        avgFy = avgFy + currentForceY;
        avgFz = avgFz + currentForceZ;
        sleep(0.1);
    }

    avgFx = (avgFx / 50);
    avgFy = (avgFy / 50);
    avgFz = (avgFz / 50);

    double updatedMaxForceX = maxForceX + avgFx;
    double updatedMaxForceY = maxForceY + avgFy;
    double updatedMaxForceZ = maxForceZ;// + avgFz;

    globalUpdatedMaxForceZ = updatedMaxForceZ;

    cout << "updatedMaxForceX = " << updatedMaxForceX << endl;
    cout << "updatedMaxForceY = " << updatedMaxForceY << endl;
    cout << "updatedMaxForceZ = " << updatedMaxForceZ << endl;

    sleep(2.0);

    contactZ = false;
    contactPressure = false;

    move_group.asyncExecute(trajectory);

    while( contactZ == false ) //currentForceZ < updatedMaxForceZ ) //&& (abs(currentForceY) < abs(updatedMaxForceY)) && (abs(currentForceZ) < abs(updatedMaxForceX)) ) 
    {

        if( (currentForceZ <= updatedMaxForceZ) )
        {

            move_group.setMaxVelocityScalingFactor(0.001);
            move_group.setMaxAccelerationScalingFactor(0.001);

            contactZ = true;

            move_group.stop();

            move_group.setStartState(*move_group.getCurrentState());
            std::vector<geometry_msgs::Pose> waypoints2;
            geometry_msgs::Pose stopPosition = move_group.getCurrentPose().pose;
//            stopPosition.position.z = stopPosition.position.z + 0.040;
            waypoints2.push_back(stopPosition);

            moveit_msgs::RobotTrajectory trajectory2;
            fraction = move_group.computeCartesianPath(waypoints2, eef_step, jump_threshold, trajectory2, true);
            ROS_INFO_NAMED("Robothon", "Visualizing plan - moving back to home position (Cartesian path) (%.2f%% acheived)", fraction * 100.0);

            move_group.execute(trajectory2);

            cout << "==================================================" << endl;
            cout << "updatedMaxForceZ = " << updatedMaxForceZ << endl;
            cout << "maximum contact force detected in Z = " << currentForceZ << endl;
            cout << "==================================================" << endl;
        }

    }

    set_speed_frac.request.speed_slider_fraction = 0.20; // change velocity
    clientSpeedSlider.call(set_speed_frac);   

    move_group.stop();
    sleep(1.0);

    move_group.setStartState(*move_group.getCurrentState());
    std::vector<geometry_msgs::Pose> waypoints3;
    waypoints3.push_back(homePositionPose);

    moveit_msgs::RobotTrajectory trajectory3;
    fraction = move_group.computeCartesianPath(waypoints3, eef_step, jump_threshold, trajectory3, true);
    ROS_INFO_NAMED("Robothon", "Visualizing plan - moving back to home position (Cartesian path) (%.2f%% acheived)", fraction * 100.0);

    move_group.execute(trajectory3);


    // End task for pressing blue button
    /*****************************************************************************************/

    sleep(2.0);

    /*****************************************************************************************/
    // Start task for removing battery lid

    // prepare z steps for sensor exploration
    incrementZaxis = 0.078;

    move_group.setStartState(*move_group.getCurrentState());
//    geometry_msgs::Pose homePositionPose = homePosition; //move_group.getCurrentPose().pose;
    std::vector<geometry_msgs::Pose> waypoints4;
    geometry_msgs::Pose pressBatteryLidPosition = homePositionPose;
    pressBatteryLidPosition.position.z = pressBatteryLidPosition.position.z - incrementZaxis;
    waypoints4.push_back(pressBatteryLidPosition);

    // We want the Cartesian path to be interpolated at a resolution of 1 cm
    // which is why we will specify 0.01 as the max step in Cartesian
    // translation.  We will specify the jump threshold as 0.0, effectively disabling it.
    // Warning - disabling the jump threshold while operating real hardware can cause
    // large unpredictable motions of redundant joints and could be a safety issue
    moveit_msgs::RobotTrajectory trajectory4;
    fraction = move_group.computeCartesianPath(waypoints4, eef_step, jump_threshold, trajectory4, true);
    ROS_INFO_NAMED("Robothon", "Visualizing plan - press battery lid (Cartesian path) (%.2f%% acheived)", fraction * 100.0);


    // Visualize the plan in RViz
    visual_tools.deleteAllMarkers();
    visual_tools.publishText(text_pose, "Cartesian Path", rvt::WHITE, rvt::XLARGE);
    visual_tools.publishPath(waypoints4, rvt::LIME_GREEN, rvt::SMALL);
    for (std::size_t i = 0; i < waypoints4.size(); ++i)
    visual_tools.publishAxisLabeled(waypoints4[i], "pt" + std::to_string(i), rvt::SMALL);
    visual_tools.trigger();

    srv.request.command_id = srv.request.COMMAND_SET_ZERO; // set force and torque values of the FT sensor 0 
    if( clientFTSensor.call( srv ) )
    {
        ROS_INFO("ret: %s", srv.response.res.c_str());
    }

    avgFx = 0.0;
    avgFy = 0.0;
    avgFz = 0.0;

    for(int i = 0; i < 50; i++)
    {
        avgFx = avgFx + currentForceX;
        avgFy = avgFy + currentForceY;
        avgFz = avgFz + currentForceZ;
        sleep(0.1);
    }

    avgFx = (avgFx / 50);
    avgFy = (avgFy / 50);
    avgFz = (avgFz / 50);

    updatedMaxForceX = maxForceX + avgFx;
    updatedMaxForceY = maxForceY + avgFy;
    updatedMaxForceZ = maxForceZ + (avgFz*1);

    globalUpdatedMaxForceZ = updatedMaxForceZ;

    cout << "updatedMaxForceX = " << updatedMaxForceX << endl;
    cout << "updatedMaxForceY = " << updatedMaxForceY << endl;
    cout << "updatedMaxForceZ = " << updatedMaxForceZ << endl;

    sleep(2.0);

    contactZ = false;

    move_group.asyncExecute(trajectory4);

    geometry_msgs::Pose stopLidPosition = move_group.getCurrentPose().pose;

    while( contactZ == false ) //currentForceZ < updatedMaxForceZ ) //&& (abs(currentForceY) < abs(updatedMaxForceY)) && (abs(currentForceZ) < abs(updatedMaxForceX)) ) 
    {

        if( (currentForceZ <= updatedMaxForceZ) )
        {
//            set_speed_frac.request.speed_slider_fraction = 0.10; // change velocity
//            clientSpeedSlider.call(set_speed_frac);   

            move_group.setMaxVelocityScalingFactor(0.01);
            move_group.setMaxAccelerationScalingFactor(0.01);

//            sleep(2.0);

            contactZ = true;

            move_group.stop();

            move_group.setStartState(*move_group.getCurrentState());
            std::vector<geometry_msgs::Pose> waypoints5;
            stopLidPosition = move_group.getCurrentPose().pose;
            cout << "Stop z position = " << stopLidPosition.position.z << endl;
            stopLidPosition.position.z = stopLidPosition.position.z - 0.0015;
            waypoints5.push_back(stopLidPosition);

            moveit_msgs::RobotTrajectory trajectory5;
            fraction = move_group.computeCartesianPath(waypoints5, eef_step, jump_threshold, trajectory5, true);
            ROS_INFO_NAMED("Robothon", "Visualizing plan - stop position (Cartesian path) (%.2f%% acheived)", fraction * 100.0);

            move_group.execute(trajectory5);

            cout << "==================================================" << endl;
            cout << "Current z position = " << stopLidPosition.position.z << endl;
            cout << "updatedMaxForceZ = " << updatedMaxForceZ << endl;
            cout << "maximum contact force detected in Z = " << currentForceZ << endl;
            cout << "==================================================" << endl;
        }

    }


    sleep(2.0);

    move_group.setStartState(*move_group.getCurrentState());
    std::vector<geometry_msgs::Pose> waypoints6;
    geometry_msgs::Pose slideLidPosition = move_group.getCurrentPose().pose;
    slideLidPosition.position.y = slideLidPosition.position.y - 0.020;
    waypoints6.push_back(slideLidPosition);

    moveit_msgs::RobotTrajectory trajectory6;
    fraction = move_group.computeCartesianPath(waypoints6, eef_step, jump_threshold, trajectory6, true);
    ROS_INFO_NAMED("Robothon", "Visualizing plan - slide lid position (Cartesian path) (%.2f%% acheived)", fraction * 100.0);

    move_group.execute(trajectory6);
    
    sleep(2.0);

    // move robot back to stopLidPosition

    move_group.setStartState(*move_group.getCurrentState());
    geometry_msgs::Pose topOfLidPosition = move_group.getCurrentPose().pose;
    std::vector<geometry_msgs::Pose> waypoints7;
//    geometry_msgs::Pose topOfLidPosition = stopLidPosition;
    topOfLidPosition.position.z = topOfLidPosition.position.z + 0.020;
    waypoints7.push_back(topOfLidPosition);

    moveit_msgs::RobotTrajectory trajectory7;
    fraction = move_group.computeCartesianPath(waypoints7, eef_step, jump_threshold, trajectory7, true);
    ROS_INFO_NAMED("Robothon", "Visualizing plan - move to top of lid position (Cartesian path) (%.2f%% acheived)", fraction * 100.0);

    move_group.execute(trajectory7);
    
    sleep(2.0);

    // rotate wrist

//    success = false;
//    const robot_state::JointModelGroup* joint_model_group2 = move_group.getCurrentState()->getJointModelGroup(PLANNING_GROUP);

    move_group.setMaxVelocityScalingFactor(0.50);
    move_group.setMaxAccelerationScalingFactor(0.50);

    // Since we set the start state we have to clear it before planning other paths
//    move_group.setStartStateToCurrentState();

//    robot_state::RobotState start_state(*move_group.getCurrentState());
    move_group.setStartState(*move_group.getCurrentState());

//    moveit::planning_interface::MoveGroupInterface::Plan plan2;
    moveit::core::RobotStatePtr current_state2 = move_group.getCurrentState();
//    std::vector<double> joint_group_positions2;
    current_state2->copyJointGroupPositions(joint_model_group, joint_group_positions);

 // move robot to home position
/*    shoulder_pan_value = -23.64;
    shoulder_lift_value = -84.94;
    elbow_value = 45.46;
    wrist_1_value = -49.61;
    wrist_2_value = -90.08;
    wrist_3_value = 240.00;
    success = false;
*/  
    for( int i = 0; i < 6; i++ )
        cout << "Join[" << i << "] = " << joint_group_positions[i] << endl;

    // start moving robot to home position
    std::map<std::string, double> graspLidPosition;

    graspLidPosition["shoulder_pan_joint"] = joint_group_positions[0];	// (deg*PI/180)
    graspLidPosition["shoulder_lift_joint"] = joint_group_positions[1];
    graspLidPosition["elbow_joint"] = joint_group_positions[2];
    graspLidPosition["wrist_1_joint"] = joint_group_positions[3];
    graspLidPosition["wrist_2_joint"] = joint_group_positions[4];
    graspLidPosition["wrist_3_joint"] = joint_group_positions[5] + (-90.0 * 3.1416 / 180);
/*
    graspLidPosition["shoulder_pan_joint"] = shoulder_pan_value * 3.1416 / 180;	// (deg*PI/180)
    graspLidPosition["shoulder_lift_joint"] = shoulder_lift_value * 3.1416 / 180;
    graspLidPosition["elbow_joint"] = elbow_value * 3.1416 / 180;
    graspLidPosition["wrist_1_joint"] = wrist_1_value * 3.1416 / 180;
    graspLidPosition["wrist_2_joint"] = wrist_2_value * 3.1416 / 180;
    graspLidPosition["wrist_3_joint"] = wrist_3_value * 3.1416 / 180; //joint_group_positions[5] + (-90.0 * 3.1416 / 180);
*/
    move_group.setJointValueTarget(graspLidPosition);
//    move_group.setPlanningTime(10.0);

    success = (move_group.plan(plan) == moveit::planning_interface::MoveItErrorCode::SUCCESS);
    ROS_INFO("Visualizing PRE-START position plan (%.2f%% acheived)",success * 100.0);

    move_group.execute(plan);
  
    move_group.setStartState(*move_group.getCurrentState());
  
    sleep(2.0);
/*
    // When done with the path constraint be sure to clear it.
    move_group.clearPathConstraints();

    // Since we set the start state we have to clear it before planning other paths
    move_group.setStartStateToCurrentState();
*/
    // open gripper

    // set gripper to standby to clear the flags
    outputControlValues.rGTO = 0;
    Robotiq2FGripperArgPub.publish(outputControlValues);
    std::cout << "STANDBY GRIPPER" << std::endl; 
    sleep(2);

    // close the gripper to the maximum value of rPR = 255
    // rGTO = 1 allows the robot to perform an action
    outputControlValues.rGTO = 1;
    outputControlValues.rSP = 100;
    outputControlValues.rPR = 125;
    outputControlValues.rFR = 100;
    Robotiq2FGripperArgPub.publish(outputControlValues);
    std::cout << "OPEN GRIPPER" << std::endl; 

    // wait until the activation action is completed to continue with the next action
//        while( gripperStatus.gOBJ != 3 && touchDetected == false )
    while( gripperStatus.gOBJ != 3 )
    {
//            printf("IN PROGRESS: gOBJ [%d]\n", gripperStatus.gOBJ);
//            usleep(1000);
    }

    printf("COMPLETED: gOBJ [%d]\n", gripperStatus.gOBJ);
    sleep(2);


    // move robot down to grasp battery lid

    move_group.setStartState(*move_group.getCurrentState());
    std::vector<geometry_msgs::Pose> waypoints8;
    geometry_msgs::Pose closeOfLidPosition = move_group.getCurrentPose().pose;;
    closeOfLidPosition.position.z = stopLidPosition.position.z - 0.018; // - 0.002;
    waypoints8.push_back(closeOfLidPosition);

    moveit_msgs::RobotTrajectory trajectory8;
    fraction = move_group.computeCartesianPath(waypoints8, eef_step, jump_threshold, trajectory8, true);
    ROS_INFO_NAMED("Robothon", "Visualizing plan - move down to lid position (Cartesian path) (%.2f%% acheived)", fraction * 100.0);

    move_group.execute(trajectory8);
    
    sleep(2.0);


    // close gripper

    // set gripper to standby to clear the flags
    outputControlValues.rGTO = 0;
    Robotiq2FGripperArgPub.publish(outputControlValues);
    std::cout << "STANDBY GRIPPER" << std::endl; 
    sleep(2);

    // close the gripper to the maximum value of rPR = 255
    // rGTO = 1 allows the robot to perform an action
    outputControlValues.rGTO = 1;
    outputControlValues.rSP = 100;
    outputControlValues.rPR = 225;
    outputControlValues.rFR = 50;
    Robotiq2FGripperArgPub.publish(outputControlValues);
    std::cout << "CLOSE GRIPPER" << std::endl; 

    // wait until the activation action is completed to continue with the next action
//        while( gripperStatus.gOBJ != 3 && touchDetected == false )
    while( gripperStatus.gOBJ != 3 && gripperStatus.gOBJ != 2)
    {
//            printf("IN PROGRESS: gOBJ [%d]\n", gripperStatus.gOBJ);
//            usleep(1000);
    }

    printf("COMPLETED: gOBJ [%d]\n", gripperStatus.gOBJ);
    sleep(2);


    // End task for removing battery lid
    /*****************************************************************************************/


    /*****************************************************************************************/
    // Start task go to home position

    // move robot up

    move_group.setStartState(*move_group.getCurrentState());
    geometry_msgs::Pose upOfLidPosition = move_group.getCurrentPose().pose;
    std::vector<geometry_msgs::Pose> waypoints12;
    upOfLidPosition.position.z = upOfLidPosition.position.z + 0.040;
    waypoints12.push_back(upOfLidPosition);

    moveit_msgs::RobotTrajectory trajectory12;
    fraction = move_group.computeCartesianPath(waypoints12, eef_step, jump_threshold, trajectory12, true);
    ROS_INFO_NAMED("Robothon", "Visualizing plan - moving up before moving to home position (Cartesian path) (%.2f%% acheived)", fraction * 100.0);

    move_group.execute(trajectory12);

    sleep(1.0);

    move_group.setStartState(*move_group.getCurrentState());
    std::vector<geometry_msgs::Pose> waypoints9;
    waypoints9.push_back(homePositionPose);

    moveit_msgs::RobotTrajectory trajectory9;
    fraction = move_group.computeCartesianPath(waypoints9, eef_step, jump_threshold, trajectory9, true);
    ROS_INFO_NAMED("Robothon", "Visualizing plan - moving back to home position (Cartesian path) (%.2f%% acheived)", fraction * 100.0);

    move_group.execute(trajectory9);

    sleep(1.0);

    // move robot down

    move_group.setStartState(*move_group.getCurrentState());
    geometry_msgs::Pose releaseLidPosition = move_group.getCurrentPose().pose;
    releaseLidPosition.position.z = releaseLidPosition.position.z - 0.110;
    std::vector<geometry_msgs::Pose> waypoints10;
    waypoints10.push_back(releaseLidPosition);

    moveit_msgs::RobotTrajectory trajectory10;
    fraction = move_group.computeCartesianPath(waypoints10, eef_step, jump_threshold, trajectory10, true);
    ROS_INFO_NAMED("Robothon", "Visualizing plan - moving release lid position (Cartesian path) (%.2f%% acheived)", fraction * 100.0);

    move_group.execute(trajectory10);

    sleep(1.0);

    // close the gripper to the maximum value of rPR = 255
    // rGTO = 1 allows the robot to perform an action
    outputControlValues.rGTO = 1;
    outputControlValues.rSP = 100;
    outputControlValues.rPR = 100;
    outputControlValues.rFR = 100;
    Robotiq2FGripperArgPub.publish(outputControlValues);
    std::cout << "OPEN GRIPPER" << std::endl; 

    // wait until the activation action is completed to continue with the next action
//        while( gripperStatus.gOBJ != 3 && touchDetected == false )
    while( gripperStatus.gOBJ != 3 )
    {
//            printf("IN PROGRESS: gOBJ [%d]\n", gripperStatus.gOBJ);
//            usleep(1000);
    }

    printf("COMPLETED: gOBJ [%d]\n", gripperStatus.gOBJ);
    sleep(2);

    // End task go to home position
    /*****************************************************************************************/


    // move robot to home position

    move_group.setStartState(*move_group.getCurrentState());
    std::vector<geometry_msgs::Pose> waypoints11;
    waypoints11.push_back(homePositionPose);

    moveit_msgs::RobotTrajectory trajectory11;
    fraction = move_group.computeCartesianPath(waypoints11, eef_step, jump_threshold, trajectory11, true);
    ROS_INFO_NAMED("Robothon", "Visualizing plan - moving back to home position (Cartesian path) (%.2f%% acheived)", fraction * 100.0);

    move_group.execute(trajectory11);


    // set gripper to standby to clear the flags
    outputControlValues.rGTO = 0;
    outputControlValues.rPR = 0;
    outputControlValues.rSP = 0;
    outputControlValues.rFR = 0;

    Robotiq2FGripperArgPub.publish(outputControlValues);
    std::cout << "STANDBY GRIPPER" << std::endl; 
    sleep(1);

    }

    // reset the robotic gripper (needed to activate the robot)
    outputControlValues.rACT = 0;
    outputControlValues.rGTO = 0;
    outputControlValues.rATR = 0;
    outputControlValues.rPR = 0;
    outputControlValues.rSP = 0;
    outputControlValues.rFR = 0;
    Robotiq2FGripperArgPub.publish(outputControlValues);
    std::cout << " GRIPPER" << std::endl; 
    usleep(1000);
    ros::waitForShutdown();

    return 0;

}
