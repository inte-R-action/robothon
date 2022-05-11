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
string repositionRobot = "nomove";
string previousRepositionRobot = "nomove";
bool robotRepositionReady = false;
double previousVelostatValues[3];
bool batteryDetected = false;

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
    avgVelostatGripper[0] = 0.0;
    avgVelostatGripper[1] = 0.0;
    avgVelostatGripper[2] = 0.0;

//    avgVelostatGripper[0] = ( msg->data[0] + msg->data[1] + msg->data[2] ) /3.0;    // edge of sensor
//    avgVelostatGripper[1] = ( msg->data[3] + msg->data[4] + msg->data[5] ) /3.0;    // centre of sensor
//    avgVelostatGripper[2] = ( msg->data[6] + msg->data[7] + msg->data[8] ) /3.0;    // edge of sensor

    avgVelostatGripper[0] = ( msg->data[0] + msg->data[1] ) / 2.0;    // edge of sensor
    avgVelostatGripper[1] = ( msg->data[3] + msg->data[4] ) / 2.0;    // centre of sensor
    avgVelostatGripper[2] = ( msg->data[6] + msg->data[7] ) / 2.0;    // edge of sensor

    int caseMovement = 0;

    if( robotRepositionReady == true )
    {
        cout << "reading velostat" << endl;
        if( avgVelostatGripper[0] >= 7.0 || avgVelostatGripper[1] >= 7.0 || avgVelostatGripper[2] >= 7.0 )
        {
            if( avgVelostatGripper[0] >= 7.0)
            {

                if( avgVelostatGripper[0] >= avgVelostatGripper[1] )
                {
                    cout << "case left" << endl;
                    caseMovement = 0;
                    batteryDetected = true;
    //                repositionRobot = "left";
                }
            }
            else if( avgVelostatGripper[2] >= 7.0)
            {

                if( avgVelostatGripper[2] >= avgVelostatGripper[1] )
                {
                    cout << "case right" << endl;
                    caseMovement = 1;
                    batteryDetected = true;
    //                repositionRobot = "right";
                }
            }
            else
            {
                if( batteryDetected == true )
                    caseMovement = rand() % 2;
                else
                {
                    cout << "case nomove" << endl;
                    caseMovement = 2;
                }
    //            repositionRobot = "nomove";
            }
        }
        else
        {
            caseMovement = rand() % 2;
            cout << "random: " << caseMovement << endl;
        }

/*        if( caseMovement == 0 )
            repositionRobot = "left";
        else if( caseMovement == 1 )
            repositionRobot = "right";

        else
            repositionRobot = previousRepositionRobot;
*/

        cout << "previous values: " << previousVelostatValues[0] << endl;
        cout << "current values: " << msg->data[3] << endl;

        if( ( msg->data[3] >= 14 ) ) //|| ( msg->data[4] >= 12.0 ) )
        {
            if( ( ( msg->data[3] - previousVelostatValues[0] ) < 0 ) ) // || ( ( msg->data[4] - previousVelostatValues[1] ) < 0 ) )
            {
                cout << "changing direction" << endl;
                cout << "changing diff value 1: " << msg->data[3] - previousVelostatValues[0] << endl;
//                cout << "changing diff value 2: " << msg->data[4] - previousVelostatValues[1] << endl;
                if( caseMovement == 0 )
                    caseMovement = 1; 
                else if( caseMovement == 1 )
                    caseMovement = 0; 
            }
            else if( ( abs( msg->data[3] - previousVelostatValues[0] ) < 1 ) ) // || ( abs( msg->data[4] - previousVelostatValues[1] ) < 1 ) )
            {
                cout << "centre previous values: " << previousVelostatValues[0] << ", " << previousVelostatValues[1] << endl;
                cout << "centre current values: " << msg->data[3] << ", " << msg->data[4] << endl;
                cout << "completed diff value 1: " << abs(msg->data[3] - previousVelostatValues[0]) << endl;
//                cout << "completed diff value 2: " << abs(msg->data[4] - previousVelostatValues[1]) << endl;
                caseMovement = 3;
                repositionRobot = "completed";
                batteryDetected = false;

                cout << "minimum distance found" << endl;
            }
        }

//        if( repositionRobot.compare("completed") )
//            previousRepositionRobot = "right";

        if( caseMovement == 0 )
            repositionRobot = "left";
        else if( caseMovement == 1 )
            repositionRobot = "right";
        else if( caseMovement == 3 )
            repositionRobot = "completed";
        else
            repositionRobot = previousRepositionRobot;

        previousVelostatValues[0] = msg->data[3];
        previousVelostatValues[1] = msg->data[4];
        previousVelostatValues[2] = msg->data[5];

        robotRepositionReady = false;
    }
}



int main(int argc, char** argv)
{
    ros::init(argc, argv, "repositioning_gripper_test");

    ros::NodeHandle node_handle;

    ros::Rate loop_rate(10000);
    ros::AsyncSpinner spinner(1);
    spinner.start();

    // connection of publisher and subscriber with the Robotiq controller from ROS Industrial
    ros::Publisher Robotiq2FGripperArgPub = node_handle.advertise<robotiq_2f_gripper_control::Robotiq2FGripper_robot_output>("Robotiq2FGripperRobotOutput", 1);
    ros::Subscriber Robotiq2FGripperStatusSub = node_handle.subscribe("Robotiq2FGripperRobotInput", 1, gripperStatusCallback);
    ros::Subscriber RobotiqFTSensor = node_handle.subscribe("robotiq_ft_sensor", 1000, ftSensorCallback);
    ros::ServiceClient clientFTSensor = node_handle.serviceClient<robotiq_ft_sensor::sensor_accessor>("robotiq_ft_sensor_acc");

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

    const double jump_threshold = 0.0;
    const double eef_step = 0.01;
    double fraction = 0.0;

    while( ros::ok() )
    {
        move_group.setStartState(*move_group.getCurrentState());

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
        std::vector<geometry_msgs::Pose> waypoints;
        geometry_msgs::Pose repositionToGraspBattery = move_group.getCurrentPose().pose;
        repositionToGraspBattery.position.z = repositionToGraspBattery.position.z - 0.040;
        waypoints.push_back(repositionToGraspBattery);

        moveit_msgs::RobotTrajectory trajectory;
        fraction = move_group.computeCartesianPath(waypoints, eef_step, jump_threshold, trajectory, true);
        ROS_INFO_NAMED("Robothon", "Visualizing plan - repositioning (Cartesian path) (%.2f%% acheived)", fraction * 100.0);

        move_group.execute(trajectory);

        sleep(0.1);



        move_group.setStartState(*move_group.getCurrentState());
        geometry_msgs::Pose homePositionPose = move_group.getCurrentPose().pose;


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

        /////////////////////////////////////////////////////////////////////////
        // start: Loop here to position the battery in the centre of the gripper


        bool isBatteryCentered = false;

        while( isBatteryCentered == false )
        {
            outputControlValues.rGTO = 0;
            Robotiq2FGripperArgPub.publish(outputControlValues);
            std::cout << "STANDBY GRIPPER" << std::endl; 
            sleep(1);

            // close the gripper to the maximum value of rPR = 255
            // rGTO = 1 allows the robot to perform an action
            outputControlValues.rGTO = 1;
            outputControlValues.rSP = 100;
            outputControlValues.rPR = 245;
            outputControlValues.rFR = 220;
            Robotiq2FGripperArgPub.publish(outputControlValues);
            std::cout << "CLOSE GRIPPER" << std::endl; 

            // wait until the activation action is completed to continue with the next action
            while( gripperStatus.gOBJ != 3  && gripperStatus.gOBJ != 2 )
            {
            }

            printf("COMPLETED: gOBJ [%d]\n", gripperStatus.gOBJ);

            sleep(0.1);

//            robotRepositionReady = true;

//            string nextRobotReposition = repositionRobot;

            robotRepositionReady = true;

            sleep(1.0);

            cout << "Left column: " << avgVelostatGripper[0] << endl;
            cout << "Middle column: " << avgVelostatGripper[1] << endl;
            cout << "Right column: " << avgVelostatGripper[2] << endl;
            cout << "reposition direction: " << repositionRobot << endl;

            string nextRobotReposition = repositionRobot;

            robotRepositionReady = false;

            move_group.setStartState(*move_group.getCurrentState());
            waypoints.clear();
            repositionToGraspBattery = move_group.getCurrentPose().pose;

            if( nextRobotReposition.compare("completed") == 0 )
            {   
                isBatteryCentered = true;
                cout << "Battery centered" << endl;
            }
            else
            {
                isBatteryCentered = false;

//                geometry_msgs::Pose repositionToGraspBattery = move_group.getCurrentPose().pose;

                if( nextRobotReposition.compare("left") == 0 )
                {
                    repositionToGraspBattery.position.x = repositionToGraspBattery.position.x - 0.002;
//                    cout << "reposition to left" << endl;
                }
                else if( nextRobotReposition.compare("right") == 0 )
                {
                    repositionToGraspBattery.position.x = repositionToGraspBattery.position.x + 0.002;
//                    cout << "reposition to right" << endl;
                }
                else
                {
                    repositionToGraspBattery.position.x = repositionToGraspBattery.position.x + 0.00;
//                    cout << "no reposition" << endl;
                }

                cout << "reposition to " << nextRobotReposition << endl;


//                robotRepositionReady = false;

                outputControlValues.rGTO = 0;
                Robotiq2FGripperArgPub.publish(outputControlValues);
                std::cout << "STANDBY GRIPPER" << std::endl; 
                sleep(1);

                // open gripper for repositioning battery
                // rGTO = 1 allows the robot to perform an action
                outputControlValues.rGTO = 1;
                outputControlValues.rSP = 100;
                outputControlValues.rPR = 150;
                outputControlValues.rFR = 200;
                Robotiq2FGripperArgPub.publish(outputControlValues);
                std::cout << "OPEN GRIPPER" << std::endl; 

                // wait until the activation action is completed to continue with the next action
                while( gripperStatus.gOBJ != 3 )
                {
                }

                printf("COMPLETED: gOBJ [%d]\n", gripperStatus.gOBJ);

                sleep(0.1);
            }

//            move_group.setStartState(*move_group.getCurrentState());
//            std::vector<geometry_msgs::Pose> waypoints;
            waypoints.push_back(repositionToGraspBattery);

//            moveit_msgs::RobotTrajectory trajectory;
            fraction = move_group.computeCartesianPath(waypoints, eef_step, jump_threshold, trajectory, true);
            ROS_INFO_NAMED("Robothon", "Visualizing plan - repositioning (Cartesian path) (%.2f%% acheived)", fraction * 100.0);

            move_group.execute(trajectory);

//            sleep(0.1);
        }

        // end: Loop here to position the battery in the centre of the gripper
        /////////////////////////////////////////////////////////////////////////


        move_group.setStartState(*move_group.getCurrentState());
        waypoints.clear();
        geometry_msgs::Pose slideBattery1Position6 = move_group.getCurrentPose().pose;
        slideBattery1Position6.position.z = slideBattery1Position6.position.z + 0.030;
        waypoints.push_back(slideBattery1Position6);

//        moveit_msgs::RobotTrajectory trajectory;
        fraction = move_group.computeCartesianPath(waypoints, eef_step, jump_threshold, trajectory, true);
        ROS_INFO_NAMED("Robothon", "Visualizing plan - moving back to home position (Cartesian path) (%.2f%% acheived)", fraction * 100.0);

        move_group.execute(trajectory);

        sleep(2);
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
