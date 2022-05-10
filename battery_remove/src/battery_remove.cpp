/*
 *********************************************************************************
 * Author: inte-R-action lab
 * Email: u.martinez@bath.ac.uk
 * Date: 05-May-2022
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
#include "std_msgs/Float32MultiArray.h"
#include "geometry_msgs/Point.h"

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

double avgVelostatGripper[3];
string repositionRobot = "";

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


void velostatSensorCallback(const std_msgs::Float32MultiArray::ConstPtr& msg)
{
/*    for(int i = 0; i < msg->data.size(); i++ )
    {
	    fileVelostat << msg->data[i];
    }
*/

    avgVelostatGripper[0] = 0.0;
    avgVelostatGripper[1] = 0.0;
    avgVelostatGripper[2] = 0.0;

    avgVelostatGripper[0] = ( msg->data[0] + msg->data[1] + msg->data[2] ) /2.0;
    avgVelostatGripper[1] = ( msg->data[3] + msg->data[4] + msg->data[5] ) /2.0;
    avgVelostatGripper[2] = ( msg->data[6] + msg->data[7] + msg->data[8] ) /2.0;

/*    if( (avgVelostatGripper[0] >= 7.0) || (avgVelostatGripper[1] >= 7.0) || (avgVelostatGripper[2] >= 7.0) )
    {

        if( avgVelostatGripper[0] >= avgVelostatGripper[1] )
            repositionRobot = "left";
        else if( avgVelostatGripper[2] >= avgVelostatGripper[1] )
            repositionRobot = "right";
        else
            repositionRobot = "nomove";
    }
    else
        repositionRobot = "nomove";
*/

    if( avgVelostatGripper[0] >= 7.0)
    {

        if( avgVelostatGripper[0] >= avgVelostatGripper[1] )
            repositionRobot = "left";
    }
    else if( avgVelostatGripper[2] >= 7.0)
    {

        if( avgVelostatGripper[2] >= avgVelostatGripper[1] )
            repositionRobot = "right";
    }
    else
        repositionRobot = "nomove";

    cout << "Velostat sensor average: " << repositionRobot << endl;

}

void pressureSensorCallback(const std_msgs::Float32::ConstPtr& msg)
{
    currentPressure = msg->data;
//    cout << "Pressure data: " << msg->data << endl;
}



int main(int argc, char** argv)
{
    ros::init(argc, argv, "battery_remove");

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

  	ros::Subscriber subVelostatData = node_handle.subscribe("velostatData", 1000, velostatSensorCallback);

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


    ur_msgs::SetSpeedSliderFraction set_speed_frac;
    set_speed_frac.request.speed_slider_fraction = 0.80; // change velocity
    clientSpeedSlider.call(set_speed_frac);   

    int b = 0;
    cin >> b;


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



    geometry_msgs::Pose slideBattery1Position1;

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

//    sleep(2);

    // wait until the activation action is completed to continue with the next action
    while( gripperStatus.gSTA != 3 )
    {
//        printf("IN PROGRESS: gSTA [%d]\n", gripperStatus.gSTA);
//        usleep(1000);
    }

    printf("COMPLETED: gSTA [%d]\n", gripperStatus.gSTA);
//    sleep(2);

    // set gripper to standby to clear the flags
    outputControlValues.rGTO = 0;
    Robotiq2FGripperArgPub.publish(outputControlValues);
    std::cout << "STANDBY GRIPPER" << std::endl; 
    sleep(1);


    // close the gripper to the maximum value of rPR = 255
    // rGTO = 1 allows the robot to perform an action
    outputControlValues.rGTO = 1;
    outputControlValues.rSP = 100;
    outputControlValues.rPR = 170;
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
//    sleep(2);

    /*****************************************************************************************/
    // Start battery1 remove

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

    bool maxThresholdZ = false;
    contactZ = false;
    contactPressure = false;

    double updatedMaxForceX = 0.0;
    double updatedMaxForceY = 0.0;
    double updatedMaxForceZ = 0.0;

    double avgFx = 0.0;
    double avgFy = 0.0;
    double avgFz = 0.0;

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
    updatedMaxForceZ = avgFz + (avgFz * 10.00); //maxForceZ + avgFz;

    if( fabs(updatedMaxForceZ) >= 1.0 && fabs(updatedMaxForceZ) <= 3.0 )
        maxThresholdZ = true;
    else
        cout << "Adjusting maxThresholZ" << endl;

    }

    if( updatedMaxForceZ > 0 )
        updatedMaxForceZ = -1 * updatedMaxForceZ;

    globalUpdatedMaxForceZ = updatedMaxForceZ;

    cout << "avgFx = " << avgFx << endl;
    cout << "avgFy = " << avgFy << endl;
    cout << "avgFx = " << avgFz << endl;

    cout << "updatedMaxForceX = " << updatedMaxForceX << endl;
    cout << "updatedMaxForceY = " << updatedMaxForceY << endl;
    cout << "updatedMaxForceZ = " << updatedMaxForceZ << endl;

    sleep(0.5);

    set_speed_frac.request.speed_slider_fraction = 0.10; // change velocity
    clientSpeedSlider.call(set_speed_frac);   

    move_group.asyncExecute(trajectory);

    srv.request.command_id = srv.request.COMMAND_SET_ZERO; // set force and torque values of the FT sensor 0 
    if( clientFTSensor.call( srv ) )
    {
        ROS_INFO("ret: %s", srv.response.res.c_str());
    }

    contactZ = false;

    while( contactZ == false ) //currentForceZ < updatedMaxForceZ ) //&& (abs(currentForceY) < abs(updatedMaxForceY)) && (abs(currentForceZ) < abs(updatedMaxForceX)) ) 
    {
        if( currentForceZ > 0 )
            currentForceZ = -1 * currentForceZ;

        if( currentForceZ <= updatedMaxForceZ )
        {
            move_group.stop();

            contactZ = true;

            move_group.setStartState(*move_group.getCurrentState());
            std::vector<geometry_msgs::Pose> waypoints2;
            geometry_msgs::Pose stopPosition = move_group.getCurrentPose().pose;
            waypoints2.push_back(stopPosition);

            slideBattery1Position1 = stopPosition;


            moveit_msgs::RobotTrajectory trajectory2;
            fraction = move_group.computeCartesianPath(waypoints2, eef_step, jump_threshold, trajectory2, true);
            ROS_INFO_NAMED("Robothon", "Visualizing plan - moving back to home position (Cartesian path) (%.2f%% acheived)", fraction * 100.0);

//            move_group.execute(trajectory2);
            move_group.asyncExecute(trajectory2);

//            set_speed_frac.request.speed_slider_fraction = 0.20; // change velocity
//            clientSpeedSlider.call(set_speed_frac);   

            sleep(1.0);

            cout << "==================================================" << endl;
            cout << "updatedMaxForceZ = " << updatedMaxForceZ << endl;
            cout << "maximum contact force detected in Z = " << currentForceZ << endl;
            cout << "==================================================" << endl;
        }
    }

//   move the tip 25 mm down after detecting the touch 
    move_group.setStartState(*move_group.getCurrentState());
    std::vector<geometry_msgs::Pose> waypoints3;
    geometry_msgs::Pose batteryPosition1 = move_group.getCurrentPose().pose;
    batteryPosition1.position.z = batteryPosition1.position.z - 0.0030;
    waypoints3.push_back(batteryPosition1);

    moveit_msgs::RobotTrajectory trajectory3;
    fraction = move_group.computeCartesianPath(waypoints3, eef_step, jump_threshold, trajectory3, true);
    ROS_INFO_NAMED("Robothon", "Visualizing plan - Battery Removing First Step (Cartesian path) (%.2f%% acheived)", fraction * 100.0);

    move_group.execute(trajectory3);

    sleep(1);

    int a = 0;
//    cin >> a ; 

// move the robot on y axis to take off the battery from the lid
    move_group.setStartState(*move_group.getCurrentState());
    std::vector<geometry_msgs::Pose> waypoints4;
    geometry_msgs::Pose batteryPosition2 = move_group.getCurrentPose().pose;
    batteryPosition2.position.y = batteryPosition2.position.y - 0.010;

    waypoints4.push_back(batteryPosition2);

    moveit_msgs::RobotTrajectory trajectory4;
    fraction = move_group.computeCartesianPath(waypoints4, eef_step, jump_threshold, trajectory4, true);
    ROS_INFO_NAMED("Robothon", "Visualizing plan - Battery Removing Second Step (Cartesian path) (%.2f%% acheived)", fraction * 100.0);

    move_group.execute(trajectory4);

    cout << "Force_X:" << currentForceX << "\t" << "Force_Y:" << currentForceY << "\t" << "Force_Z:" << currentForceZ << endl;

    sleep(1);

//    a = 0;
//    cin >> a ; 

   // set_speed_frac.request.speed_slider_fraction = 0.80; // change velocity
   // clientSpeedSlider.call(set_speed_frac);   

//    move_group.stop();
    sleep(0.5);
   
    
    move_group.setStartState(*move_group.getCurrentState());
    std::vector<geometry_msgs::Pose> waypoints8;
    geometry_msgs::Pose batteryPosition3 = move_group.getCurrentPose().pose;
    batteryPosition3.position.z = batteryPosition3.position.z + 0.040;
    waypoints8.push_back(batteryPosition3);

    moveit_msgs::RobotTrajectory trajectory8;
    fraction = move_group.computeCartesianPath(waypoints8, eef_step, jump_threshold, trajectory8, true);
    ROS_INFO_NAMED("Robothon", "Visualizing plan - moving back to home position (Cartesian path) (%.2f%% acheived)", fraction * 100.0);

    move_group.execute(trajectory8);

    sleep(0.5);
// end of battery1 remove


    /********/
    // start sliding battery 1 process

    // close the gripper to the maximum value of rPR = 255
    // rGTO = 1 allows the robot to perform an action
    outputControlValues.rGTO = 1;
    outputControlValues.rSP = 100;
    outputControlValues.rPR = 230;
    outputControlValues.rFR = 200;
    Robotiq2FGripperArgPub.publish(outputControlValues);
    std::cout << "CLOSE GRIPPER" << std::endl; 

    // wait until the activation action is completed to continue with the next action
//        while( gripperStatus.gOBJ != 3 && touchDetected == false )
    while( gripperStatus.gOBJ != 3 )
    {
    }

    printf("COMPLETED: gOBJ [%d]\n", gripperStatus.gOBJ);


    //    robot_state::RobotState start_state(*move_group.getCurrentState());
    move_group.setStartState(*move_group.getCurrentState());

//    moveit::planning_interface::MoveGroupInterface::Plan plan2;
    moveit::core::RobotStatePtr current_state2 = move_group.getCurrentState();
//    std::vector<double> joint_group_positions2;
    current_state2->copyJointGroupPositions(joint_model_group, joint_group_positions);

 
    for( int i = 0; i < 6; i++ )
        cout << "Join[" << i << "] = " << joint_group_positions[i] << endl;

    // start moving robot to home position
    std::map<std::string, double> graspLidPosition;

    graspLidPosition["shoulder_pan_joint"] = joint_group_positions[0];	// (deg*PI/180)
    graspLidPosition["shoulder_lift_joint"] = joint_group_positions[1];
    graspLidPosition["elbow_joint"] = joint_group_positions[2];
    graspLidPosition["wrist_1_joint"] = joint_group_positions[3];
    graspLidPosition["wrist_2_joint"] = joint_group_positions[4];
    graspLidPosition["wrist_3_joint"] = joint_group_positions[5] + (-180.0 * 3.1416 / 180);


    move_group.setJointValueTarget(graspLidPosition);
//    move_group.setPlanningTime(10.0);

    success = (move_group.plan(plan) == moveit::planning_interface::MoveItErrorCode::SUCCESS);
    ROS_INFO("Visualizing PRE-START position plan (%.2f%% acheived)",success * 100.0);

    move_group.execute(plan);

//    move_group.setStartState(*move_group.getCurrentState());
    sleep(1.0);


    incrementZaxis = 0.078;

    move_group.setStartState(*move_group.getCurrentState());
//    geometry_msgs::Pose homePositionPose = homePosition; //move_group.getCurrentPose().pose;

    waypoints8.clear();
    pressButtonPosition = move_group.getCurrentPose().pose;
    pressButtonPosition.position.z = pressButtonPosition.position.z - incrementZaxis;
    waypoints8.push_back(pressButtonPosition);

    // We want the Cartesian path to be interpolated at a resolution of 1 cm
    // which is why we will specify 0.01 as the max step in Cartesian
    // translation.  We will specify the jump threshold as 0.0, effectively disabling it.
    // Warning - disabling the jump threshold while operating real hardware can cause
    // large unpredictable motions of redundant joints and could be a safety issue
    //moveit_msgs::RobotTrajectory trajectory8;
    fraction = move_group.computeCartesianPath(waypoints8, eef_step, jump_threshold, trajectory, true);
    ROS_INFO_NAMED("Robothon", "Visualizing plan - press blue button (Cartesian path) (%.2f%% acheived)", fraction * 100.0);

    // Visualize the plan in RViz
    visual_tools.deleteAllMarkers();
    visual_tools.publishText(text_pose, "Cartesian Path", rvt::WHITE, rvt::XLARGE);
    visual_tools.publishPath(waypoints8, rvt::LIME_GREEN, rvt::SMALL);
    for (std::size_t i = 0; i < waypoints8.size(); ++i)
    visual_tools.publishAxisLabeled(waypoints8[i], "pt" + std::to_string(i), rvt::SMALL);
    visual_tools.trigger();

    maxThresholdZ = false;
    contactZ = false;
    contactPressure = false;

    updatedMaxForceX = 0.0;
    updatedMaxForceY = 0.0;
    updatedMaxForceZ = 0.0;

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
    updatedMaxForceZ = avgFz + (avgFz * 10.00); //maxForceZ + avgFz;trajectory

    if( fabs(updatedMaxForceZ) >= 1.0 && fabs(updatedMaxForceZ) <= 3.0 )
        maxThresholdZ = true;
    else
        cout << "Adjusting maxThresholZ" << endl;

    }

    if( updatedMaxForceZ > 0 )
        updatedMaxForceZ = -1 * updatedMaxForceZ;

    globalUpdatedMaxForceZ = updatedMaxForceZ;

    cout << "avgFx = " << avgFx << endl;
    cout << "avgFy = " << avgFy << endl;
    cout << "avgFx = " << avgFz << endl;

    cout << "updatedMaxForceX = " << updatedMaxForceX << endl;
    cout << "updatedMaxForceY = " << updatedMaxForceY << endl;
    cout << "updatedMaxForceZ = " << updatedMaxForceZ << endl;

    sleep(0.5);

    set_speed_frac.request.speed_slider_fraction = 0.10; // change velocity
    clientSpeedSlider.call(set_speed_frac);   

    move_group.asyncExecute(trajectory);

    srv.request.command_id = srv.request.COMMAND_SET_ZERO; // set force and torque values of the FT sensor 0 
    if( clientFTSensor.call( srv ) )
    {
        ROS_INFO("ret: %s", srv.response.res.c_str());
    }

    contactZ = false;

    geometry_msgs::Pose slideBattery1PositionN;

    while( contactZ == false ) //currentForceZ < updatedMaxForceZ ) //&& (abs(currentForceY) < abs(updatedMaxForceY)) && (abs(currentForceZ) < abs(updatedMaxForceX)) ) 
    {
        if( currentForceZ > 0 )
            currentForceZ = -1 * currentForceZ;

        if( currentForceZ <= updatedMaxForceZ )
        {
            move_group.stop();

            contactZ = true;

            move_group.setStartState(*move_group.getCurrentState());
            std::vector<geometry_msgs::Pose> waypoints2;
            geometry_msgs::Pose stopPosition = move_group.getCurrentPose().pose;
            waypoints2.push_back(stopPosition);

            slideBattery1PositionN = stopPosition;


            moveit_msgs::RobotTrajectory trajectory2;
            fraction = move_group.computeCartesianPath(waypoints2, eef_step, jump_threshold, trajectory2, true);
            ROS_INFO_NAMED("Robothon", "Visualizing plan - moving back to home position (Cartesian path) (%.2f%% acheived)", fraction * 100.0);

//            move_group.execute(trajectory2);
            move_group.asyncExecute(trajectory2);

//            set_speed_frac.request.speed_slider_fraction = 0.20; // change velocity
//            clientSpeedSlider.call(set_speed_frac);   

            sleep(1.0);

            cout << "==================================================" << endl;
            cout << "updatedMaxForceZ = " << updatedMaxForceZ << endl;
            cout << "maximum contact force detected in Z = " << currentForceZ << endl;
            cout << "==================================================" << endl;
        }
    }


/*    move_group.setStartState(*move_group.getCurrentState());
    waypoints8.clear();
    slideBattery1Position1.position.z = slideBattery1Position1.position.z + 0.005;
    waypoints8.push_back(slideBattery1Position1);

    fraction = move_group.computeCartesianPath(waypoints8, eef_step, jump_threshold, trajectory8, true);
    ROS_INFO_NAMED("Robothon", "Visualizing plan - moving back to home position (Cartesian path) (%.2f%% acheived)", fraction * 100.0);

    move_group.execute(trajectory8);
*/
    sleep(2);    

    move_group.setStartState(*move_group.getCurrentState());
    waypoints8.clear();
    geometry_msgs::Pose slideBattery1Position2 = move_group.getCurrentPose().pose;
    slideBattery1Position2.position.y = slideBattery1Position2.position.y + 0.025;
    waypoints8.push_back(slideBattery1Position2);

    fraction = move_group.computeCartesianPath(waypoints8, eef_step, jump_threshold, trajectory8, true);
    ROS_INFO_NAMED("Robothon", "Visualizing plan - moving back to home position (Cartesian path) (%.2f%% acheived)", fraction * 100.0);

    move_group.execute(trajectory8);

    sleep(2);

    move_group.setStartState(*move_group.getCurrentState());
    waypoints8.clear();
    geometry_msgs::Pose slideBattery1Position3 = move_group.getCurrentPose().pose;
    slideBattery1Position3.position.z = slideBattery1Position3.position.z + 0.040;
    waypoints8.push_back(slideBattery1Position3);

    fraction = move_group.computeCartesianPath(waypoints8, eef_step, jump_threshold, trajectory8, true);
    ROS_INFO_NAMED("Robothon", "Visualizing plan - moving back to home position (Cartesian path) (%.2f%% acheived)", fraction * 100.0);

    move_group.execute(trajectory8);
    sleep(2);


    // close the gripper to the maximum value of rPR = 255
    // rGTO = 1 allows the robot to perform an action
    outputControlValues.rGTO = 1;
    outputControlValues.rSP = 100;
    outputControlValues.rPR = 170;
    outputControlValues.rFR = 200;
    Robotiq2FGripperArgPub.publish(outputControlValues);
    std::cout << "OPEN GRIPPER" << std::endl; 

    // wait until the activation action is completed to continue with the next action
//        while( gripperStatus.gOBJ != 3 && touchDetected == false )
    while( gripperStatus.gOBJ != 3 )
    {
    }

    printf("COMPLETED: gOBJ [%d]\n", gripperStatus.gOBJ);

    sleep(2);

    move_group.setStartState(*move_group.getCurrentState());
    waypoints8.clear();
    geometry_msgs::Pose slideBattery1Position4 = move_group.getCurrentPose().pose;
    slideBattery1Position4.position.z = slideBattery1Position4.position.z - 0.040;
    waypoints8.push_back(slideBattery1Position4);

    fraction = move_group.computeCartesianPath(waypoints8, eef_step, jump_threshold, trajectory8, true);
    ROS_INFO_NAMED("Robothon", "Visualizing plan - moving back to home position (Cartesian path) (%.2f%% acheived)", fraction * 100.0);

    move_group.execute(trajectory8);

    sleep(2);

    move_group.setStartState(*move_group.getCurrentState());
    waypoints8.clear();
    geometry_msgs::Pose slideBattery1Position5 = move_group.getCurrentPose().pose;
    slideBattery1Position5.position.y = slideBattery1Position5.position.y - 0.010;
    waypoints8.push_back(slideBattery1Position5);

    fraction = move_group.computeCartesianPath(waypoints8, eef_step, jump_threshold, trajectory8, true);
    ROS_INFO_NAMED("Robothon", "Visualizing plan - moving back to home position (Cartesian path) (%.2f%% acheived)", fraction * 100.0);

    move_group.execute(trajectory8);

    sleep(2);

    /////////////////////////////////////////////////////////////////////////
    // start: Loop here to position the battery in the centre of the gripper

    bool isBatteryCentered = false;

    while( isBatteryCentered == false )
    {
        // close the gripper to the maximum value of rPR = 255
        // rGTO = 1 allows the robot to perform an action
        outputControlValues.rGTO = 1;
        outputControlValues.rSP = 100;
        outputControlValues.rPR = 230;
        outputControlValues.rFR = 200;
        Robotiq2FGripperArgPub.publish(outputControlValues);
        std::cout << "CLOSE GRIPPER" << std::endl; 

        // wait until the activation action is completed to continue with the next action
        while( gripperStatus.gOBJ != 3  && gripperStatus.gOBJ != 2 )
        {
        }

        printf("COMPLETED: gOBJ [%d]\n", gripperStatus.gOBJ);

        sleep(0.5);

        if( repositionRobot.compare("nomove") == 0 )
        {
            isBatteryCentered = true;
            cout << "Battery centered" << endl;
        }
        else
        {
            isBatteryCentered = false;

            // open gripper for repositioning battery
            // rGTO = 1 allows the robot to perform an action
            outputControlValues.rGTO = 1;
            outputControlValues.rSP = 100;
            outputControlValues.rPR = 170;
            outputControlValues.rFR = 200;
            Robotiq2FGripperArgPub.publish(outputControlValues);
            std::cout << "OPEN GRIPPER" << std::endl; 

            // wait until the activation action is completed to continue with the next action
            while( gripperStatus.gOBJ != 3  && gripperStatus.gOBJ != 2 )
            {
            }

            printf("COMPLETED: gOBJ [%d]\n", gripperStatus.gOBJ);

            sleep(1.0);


            move_group.setStartState(*move_group.getCurrentState());
            waypoints8.clear();
            geometry_msgs::Pose repositionToGraspBattery = move_group.getCurrentPose().pose;

            if( repositionRobot.compare("left") == 0 )
            {
                repositionToGraspBattery.position.y = repositionToGraspBattery.position.y - 0.005;
                cout << "reposition to left" << endl;
            }
            else if( repositionRobot.compare("right") == 0 )
            {
                repositionToGraspBattery.position.y = repositionToGraspBattery.position.y + 0.005;
                cout << "reposition to right" << endl;
            }
            else
            {
                repositionToGraspBattery.position.y = repositionToGraspBattery.position.y + 0.0;
                cout << "no reposition" << endl;
                isBatteryCentered = true;
            }

            waypoints8.push_back(repositionToGraspBattery);

            fraction = move_group.computeCartesianPath(waypoints8, eef_step, jump_threshold, trajectory8, true);
            ROS_INFO_NAMED("Robothon", "Visualizing plan - repositioning (Cartesian path) (%.2f%% acheived)", fraction * 100.0);

            move_group.execute(trajectory8);

            sleep(1.0);
        }

    }


    // end: Loop here to position the battery in the centre of the gripper
    /////////////////////////////////////////////////////////////////////////


    sleep(2);

    move_group.setStartState(*move_group.getCurrentState());
    waypoints8.clear();
    geometry_msgs::Pose slideBattery1Position6 = move_group.getCurrentPose().pose;
    slideBattery1Position6.position.z = slideBattery1Position6.position.z + 0.030;
    waypoints8.push_back(slideBattery1Position6);

    fraction = move_group.computeCartesianPath(waypoints8, eef_step, jump_threshold, trajectory8, true);
    ROS_INFO_NAMED("Robothon", "Visualizing plan - moving back to home position (Cartesian path) (%.2f%% acheived)", fraction * 100.0);

    move_group.execute(trajectory8);


    a = 0;
    cin >> a;


    /********/


// Rotate the end-effector to remove the other battery

    //    robot_state::RobotState start_state(*move_group.getCurrentState());
    move_group.setStartState(*move_group.getCurrentState());

//    moveit::planning_interface::MoveGroupInterface::Plan plan2;
    current_state2 = move_group.getCurrentState();
//    std::vector<double> joint_group_positions2;
    current_state2->copyJointGroupPositions(joint_model_group, joint_group_positions);

 
    for( int i = 0; i < 6; i++ )
        cout << "Join[" << i << "] = " << joint_group_positions[i] << endl;

    // start moving robot to home position
    graspLidPosition.clear();

    graspLidPosition["shoulder_pan_joint"] = joint_group_positions[0];	// (deg*PI/180)
    graspLidPosition["shoulder_lift_joint"] = joint_group_positions[1];
    graspLidPosition["elbow_joint"] = joint_group_positions[2];
    graspLidPosition["wrist_1_joint"] = joint_group_positions[3];
    graspLidPosition["wrist_2_joint"] = joint_group_positions[4];
    graspLidPosition["wrist_3_joint"] = joint_group_positions[5] + (-180.0 * 3.1416 / 180);


    move_group.setJointValueTarget(graspLidPosition);
//    move_group.setPlanningTime(10.0);

    success = (move_group.plan(plan) == moveit::planning_interface::MoveItErrorCode::SUCCESS);
    ROS_INFO("Visualizing PRE-START position plan (%.2f%% acheived)",success * 100.0);

    move_group.execute(plan);

    move_group.setStartState(*move_group.getCurrentState());
    sleep(1.0);
    
// battery2 removing

// prepare z steps for sensor exploration
    incrementZaxis = 0.078;

    move_group.setStartState(*move_group.getCurrentState());
//    geometry_msgs::Pose homePositionPose = homePosition; //move_group.getCurrentPose().pose;

    std::vector<geometry_msgs::Pose> waypoints20;
    pressButtonPosition = move_group.getCurrentPose().pose;
    pressButtonPosition.position.z = pressButtonPosition.position.z - incrementZaxis;
    waypoints20.push_back(pressButtonPosition);

    // We want the Cartesian path to be interpolated at a resolution of 1 cm
    // which is why we will specify 0.01 as the max step in Cartesian
    // translation.  We will specify the jump threshold as 0.0, effectively disabling it.
    // Warning - disabling the jump threshold while operating real hardware can cause
    // large unpredictable motions of redundant joints and could be a safety issue
    moveit_msgs::RobotTrajectory trajectory20;
    fraction = move_group.computeCartesianPath(waypoints20, eef_step, jump_threshold, trajectory20, true);
    ROS_INFO_NAMED("Robothon", "Visualizing plan - press blue button (Cartesian path) (%.2f%% acheived)", fraction * 100.0);

    // Visualize the plan in RViz
    visual_tools.deleteAllMarkers();
    visual_tools.publishText(text_pose, "Cartesian Path", rvt::WHITE, rvt::XLARGE);
    visual_tools.publishPath(waypoints20, rvt::LIME_GREEN, rvt::SMALL);
    for (std::size_t i = 0; i < waypoints20.size(); ++i)
    visual_tools.publishAxisLabeled(waypoints20[i], "pt" + std::to_string(i), rvt::SMALL);
    visual_tools.trigger();
    // Cartesian motions should often be slow, e.g. when approaching objects. The speed of cartesian
    // plans cannot currently be set through the maxVelocityScalingFactor, but requires you to time
    // the trajectory manually, as described [here](https://groups.google.com/forum/#!topic/moveit-users/MOoFxy2exT4).
    // Pull requests are welcome.

    // You can execute a trajectory like this.
//    move_group.execute(trajectory);
//    geometry_msgs::Pose maxPositionZ = move_group.getCurrentPose().pose;

    maxThresholdZ = false;
    contactZ = false;
    contactPressure = false;

    updatedMaxForceX = 0.0;
    updatedMaxForceY = 0.0;
    updatedMaxForceZ = 0.0;

    avgFx = 0.0;
    avgFy = 0.0;
    avgFz = 0.0;

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
    updatedMaxForceZ = avgFz + (avgFz * 10.00); //maxForceZ + avgFz;

    if( fabs(updatedMaxForceZ) >= 1.0 && fabs(updatedMaxForceZ) <= 3.0 )
        maxThresholdZ = true;
    else
        cout << "Adjusting maxThresholZ" << endl;

    }

    if( updatedMaxForceZ > 0 )
        updatedMaxForceZ = -1 * updatedMaxForceZ;

    globalUpdatedMaxForceZ = updatedMaxForceZ;

    cout << "avgFx = " << avgFx << endl;
    cout << "avgFy = " << avgFy << endl;
    cout << "avgFx = " << avgFz << endl;

    cout << "updatedMaxForceX = " << updatedMaxForceX << endl;
    cout << "updatedMaxForceY = " << updatedMaxForceY << endl;
    cout << "updatedMaxForceZ = " << updatedMaxForceZ << endl;

    sleep(0.5);

    set_speed_frac.request.speed_slider_fraction = 0.10; // change velocity
    clientSpeedSlider.call(set_speed_frac);   

    move_group.asyncExecute(trajectory20);

    srv.request.command_id = srv.request.COMMAND_SET_ZERO; // set force and torque values of the FT sensor 0 
    if( clientFTSensor.call( srv ) )
    {
        ROS_INFO("ret: %s", srv.response.res.c_str());
    }

    contactZ = false;

    while( contactZ == false ) //currentForceZ < updatedMaxForceZ ) //&& (abs(currentForceY) < abs(updatedMaxForceY)) && (abs(currentForceZ) < abs(updatedMaxForceX)) ) 
    {
        if( currentForceZ > 0 )
            currentForceZ = -1 * currentForceZ;

        if( currentForceZ <= updatedMaxForceZ )
        {
            move_group.stop();

            contactZ = true;

            move_group.setStartState(*move_group.getCurrentState());
            std::vector<geometry_msgs::Pose> waypoints21;
            geometry_msgs::Pose stopPosition = move_group.getCurrentPose().pose;
            waypoints21.push_back(stopPosition);

            moveit_msgs::RobotTrajectory trajectory21;
            fraction = move_group.computeCartesianPath(waypoints21, eef_step, jump_threshold, trajectory21, true);
            ROS_INFO_NAMED("Robothon", "Visualizing plan - moving back to home position (Cartesian path) (%.2f%% acheived)", fraction * 100.0);

//            move_group.execute(trajectory2);
            move_group.asyncExecute(trajectory21);

//            set_speed_frac.request.speed_slider_fraction = 0.20; // change velocity
//            clientSpeedSlider.call(set_speed_frac);   

            sleep(1.0);

            cout << "==================================================" << endl;
            cout << "updatedMaxForceZ = " << updatedMaxForceZ << endl;
            cout << "maximum contact force detected in Z = " << currentForceZ << endl;
            cout << "==================================================" << endl;
        }
    }

//   move the tip 25 mm down after detecting the touch 
    move_group.setStartState(*move_group.getCurrentState());
    std::vector<geometry_msgs::Pose> waypoints22;
    geometry_msgs::Pose battery2Position1 = move_group.getCurrentPose().pose;
    battery2Position1.position.z = battery2Position1.position.z - 0.0030;
    waypoints22.push_back(battery2Position1);

    moveit_msgs::RobotTrajectory trajectory22;
    fraction = move_group.computeCartesianPath(waypoints22, eef_step, jump_threshold, trajectory22, true);
    ROS_INFO_NAMED("Robothon", "Visualizing plan - Battery Removing First Step (Cartesian path) (%.2f%% acheived)", fraction * 100.0);

    move_group.execute(trajectory22);
    a = 0;
    cin >> a ; 

// move the robot on y axis to take off the battery from the lid
    move_group.setStartState(*move_group.getCurrentState());
    std::vector<geometry_msgs::Pose> waypoints23;
    geometry_msgs::Pose battery2Position2 = move_group.getCurrentPose().pose;
    battery2Position2.position.y = battery2Position2.position.y + 0.010;

    waypoints23.push_back(battery2Position2);

    moveit_msgs::RobotTrajectory trajectory23;
    fraction = move_group.computeCartesianPath(waypoints23, eef_step, jump_threshold, trajectory23, true);
    ROS_INFO_NAMED("Robothon", "Visualizing plan - Battery Removing Second Step (Cartesian path) (%.2f%% acheived)", fraction * 100.0);

    move_group.execute(trajectory23);

    cout << "Force_X:" << currentForceX << "\t" << "Force_Y:" << currentForceY << "\t" << "Force_Z:" << currentForceZ << endl;

    a = 0;
    cin >> a ; 

   // set_speed_frac.request.speed_slider_fraction = 0.80; // change velocity
   // clientSpeedSlider.call(set_speed_frac);   

//    move_group.stop();
    sleep(0.5);
   
    
    move_group.setStartState(*move_group.getCurrentState());
    std::vector<geometry_msgs::Pose> waypoints24;
    geometry_msgs::Pose battery2Position3 = move_group.getCurrentPose().pose;
    battery2Position3.position.z = battery2Position3.position.z + 0.040;
    waypoints24.push_back(battery2Position3);

    moveit_msgs::RobotTrajectory trajectory24;
    fraction = move_group.computeCartesianPath(waypoints24, eef_step, jump_threshold, trajectory24, true);
    ROS_INFO_NAMED("Robothon", "Visualizing plan - moving back to home position (Cartesian path) (%.2f%% acheived)", fraction * 100.0);

    move_group.execute(trajectory24);

    sleep(0.5);

//end of battery2 removing
    
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
