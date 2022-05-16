/*
 *********************************************************************************
 * Author: Uriel Martinez-Hernandez
 * Email: u.martinez@bath.ac.uk
 * Date: 5-February-2021
 *
 * University of Bath
 * Multimodal Interaction and Robotic Active Perception (inte-R-action) Lab
 * Centre for Autonomous Robotics (CENTAUR)
 * Department of Electronics and Electrical Engineering
 *
 * Description:
 *
 *********************************************************************************
 */

//#include <moveit_core/robot_trajectory.h>
#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>
#include <moveit_msgs/DisplayRobotState.h>
#include <moveit_msgs/DisplayTrajectory.h>
#include <moveit_msgs/AttachedCollisionObject.h>
#include <moveit_msgs/CollisionObject.h>
#include <moveit_visual_tools/moveit_visual_tools.h>
#include <moveit/trajectory_processing/iterative_time_parameterization.h>
#include "std_msgs/String.h"
#include <iostream>
#include <unistd.h>
#include <map>
#include <sstream>
#include "robotiq_ft_sensor/ft_sensor.h"
#include "robotiq_ft_sensor/sensor_accessor.h"
#include <ros/ros.h>
#include <moveit_msgs/ExecuteTrajectoryActionResult.h>
#include <tf2_ros/transform_listener.h>
#include <geometry_msgs/TransformStamped.h>
#include <tf2_ros/static_transform_broadcaster.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <test_ik_planning/object_state.h>

using namespace std;

string objectString = "";
test_ik_planning::object_state object_state_msg;
bool robotMove = false;

namespace rvt = rviz_visual_tools;
int robot_execute_code;
double ft_readings [6];

void ftSensorCallback(const robotiq_ft_sensor::ft_sensor& msg)
{
    double data [] = {msg.Fx,msg.Fy,msg.Fz,msg.Mx,msg.My,msg.Mz};
    std::copy(data, data + 6, ft_readings);
}

void robotMoveCallback(const std_msgs::String::ConstPtr& msg)
{
//  ROS_INFO("I heard: [%s]", msg->data);
 
 	//objectString.clear();

    objectString = msg->data; 

    //cout << "Robot move: " << robotMove << endl; 
    //cout << "Object: " << objectString << endl;
}

void robotExecuteCallback(const moveit_msgs::ExecuteTrajectoryActionResult::ConstPtr& msg)
{
    robot_execute_code = msg->result.error_code.val;
}

void objectDetectionCallback(const test_ik_planning::object_state::ConstPtr& msg)
{
    if ( objectString != "" ){
    if ( msg->Obj_type == objectString.substr(5) ){
        object_state_msg.Id = msg->Id;
        object_state_msg.Obj_type = msg->Obj_type;
        object_state_msg.Pose = msg->Pose;
        object_state_msg.Header = msg->Header;
    }
}
}

// Map high level position to joint angles as seen on teach pendant
// Order is: shoulder_pan_joint, shoulder_lift_joint, elbow_joint, wrist_1_joint, wrist_2_joint, wrist_3_joint
struct jnt_angs{double angles[6];};
std::map<std::string, jnt_angs> create_joint_pos(){
    std::map<std::string, jnt_angs> joint_positions;
    joint_positions["home"] = {-8.49, -101.10, 55.46, -43.0, -90.0, 0.0};//{-11.75, -83.80, 47.90, -125.0, -90.0, 0.0};
    joint_positions["look_for_objects"] = {-8.49, -101.10, 55.46, -43.0, -90.0, 0.0};
    return joint_positions;
};

class moveit_robot {
    public:
        // Setup
        // MoveIt! operates on sets of joints called "planning groups" and stores them in an object called
        // the `JointModelGroup`. Throughout MoveIt! the terms "planning group" and "joint model group"
        // are used interchangably.
        const std::string PLANNING_GROUP;

        // The :move_group_interface:`MoveGroup` class can be easily
        // setup using just the name of the planning group you would like to control and plan for.
        moveit::planning_interface::MoveGroupInterface move_group;

        // We will use the :planning_interface:`PlanningSceneInterface`
        // class to add and remove collision objects in our "virtual world" scene
        moveit::planning_interface::PlanningSceneInterface planning_scene_interface;

        const robot_state::JointModelGroup* joint_model_group;

        // Now, we call the planner to compute the plan and visualize it.
        // The plan variable contains the movements that the robot will perform to move
        // from one point to another
        moveit::planning_interface::MoveGroupInterface::Plan plan;

        moveit::core::RobotStatePtr current_state;
        std::vector<double> joint_group_positions;

        // Visualization
        // The package MoveItVisualTools provides many capabilties for visualizing objects, robots,
        // and trajectories in RViz as well as debugging tools such as step-by-step introspection of a script
        moveit_visual_tools::MoveItVisualTools visual_tools;
        Eigen::Isometry3d text_pose;

        // Joint positions map
        std::map<std::string, jnt_angs> joint_positions;

        ros::Subscriber robot_execute_sub;

        std_msgs::String  robot_status_msg;
        ros::Publisher robot_status_pub;
        ros::Publisher target_pose_pub;
        geometry_msgs::PoseStamped pose2plan2;

        // Force sensor
        ros::ServiceClient ft_client;
        ros::Subscriber ft_sub1;
        robotiq_ft_sensor::sensor_accessor ft_srv;

        moveit_robot(ros::NodeHandle* node_handle);
        void move_robot(std::map<std::string, double> targetJoints, std::string robot_action, std::string jnt_pos_name);
        void z_move(double dist, double max_velocity_scale_factor);
        bool plan_to_pose(geometry_msgs::Pose pose);
        void move_cartesian(double dist1, double dist2, double dist3);
        void basic_cartesian_move(double dist1, double dist2, double dist3);
        geometry_msgs::Pose transform_pose(geometry_msgs::Pose input_pose);
    
    private:
        ros::NodeHandle nh_; // we will need this, to pass between "main" and constructor

};

moveit_robot::moveit_robot(ros::NodeHandle* node_handle) : nh_(*node_handle), PLANNING_GROUP("manipulator"), visual_tools("world"), move_group(moveit::planning_interface::MoveGroupInterface(PLANNING_GROUP)) {

    // Raw pointers are frequently used to refer to the planning group for improved performance.
    joint_model_group = move_group.getCurrentState()->getJointModelGroup(PLANNING_GROUP);

    visual_tools.deleteAllMarkers();

    // Remote control is an introspection tool that allows users to step through a high level script
    // via buttons and keyboard shortcuts in RViz
    visual_tools.loadRemoteControl();

    // RViz provides many types of markers, in this demo we will use text, cylinders, and spheres
    //Eigen::Affine3d text_pose = Eigen::Affine3d::Identity();
    //Eigen::Isometry3d 
    text_pose = Eigen::Isometry3d::Identity();
    text_pose.translation().z() = 1.75;
    visual_tools.publishText(text_pose, "HRI Static Demo - v 0.1.0", rvt::WHITE, rvt::XLARGE);

    // Batch publishing is used to reduce the number of messages being sent to RViz for large visualizations
    visual_tools.trigger();

    // Getting Basic Information
    // 
    // We can print the name of the reference frame for this robot.
    ROS_INFO_NAMED("UR3 robot", "Reference frame: %s", move_group.getPlanningFrame().c_str());

    // We can also print the name of the end-effector link for this group.
    ROS_INFO_NAMED("UR3 robot", "End effector link: %s", move_group.getEndEffectorLink().c_str());

    // We can get a list of all the groups in the robot:
    ROS_INFO_NAMED("tutorial", "Available Planning Groups:");
    std::copy(move_group.getJointModelGroupNames().begin(), move_group.getJointModelGroupNames().end(),
            std::ostream_iterator<std::string>(std::cout, ", "));


    // Now, we call the planner to compute the plan and visualize it.
    // The plan variable contains the movements that the robot will perform to move
    // from one point to another
    //moveit::planning_interface::MoveGroupInterface::Plan plan;

    // To start, we'll create an pointer that references the current robot's state.
    // RobotState is the object that contains all the current position/velocity/acceleration data.
    moveit::core::RobotStatePtr current_state = move_group.getCurrentState();

    // Next get the current set of joint values for the group.
    std::vector<double> joint_group_positions;
    current_state->copyJointGroupPositions(joint_model_group, joint_group_positions);

    // We lower the allowed maximum velocity and acceleration to 5% of their maximum.
    // The default values are 10% (0.1).
    // Set your preferred defaults in the joint_limits.yaml file of your robot's moveit_config
    // or set explicit factors in your code if you need your robot to move faster.
    move_group.setMaxVelocityScalingFactor(0.25);
    move_group.setMaxAccelerationScalingFactor(0.15);

    joint_positions = create_joint_pos();

    robot_status_pub = nh_.advertise<std_msgs::String>("RobotStatus", 10);
    target_pose_pub = nh_.advertise<geometry_msgs::PoseStamped>("TargetPose", 10);
    pose2plan2.header.seq = 0;
    pose2plan2.header.stamp = ros::Time::now();
    pose2plan2.header.frame_id= "plan2pose";

    robot_execute_sub = nh_.subscribe("execute_trajectory/result", 1, robotExecuteCallback);

    ft_client = nh_.serviceClient<robotiq_ft_sensor::sensor_accessor>("robotiq_ft_sensor_acc");
    ft_sub1 = nh_.subscribe("robotiq_ft_sensor",100,ftSensorCallback);

    // Add collision objects
    // ^^^^^^^^^^^^^^^^^^^^^^^^^^

    ros::Publisher planning_scene_diff_publisher = nh_.advertise<moveit_msgs::PlanningScene>("planning_scene", 1);
    ros::WallDuration sleep_t(0.5);
    while ((planning_scene_diff_publisher.getNumSubscribers() < 1) and ros::ok())
    {
        sleep_t.sleep();
    }

    moveit_msgs::PlanningScene planning_scene;
    //planning_scene.robot_state.attached_collision_objects.clear();
    //planning_scene.world.collision_objects.clear();
    //planning_scene_diff_publisher.publish(planning_scene);
    std::vector<std::string> object_ids;
    object_ids.push_back("gripper");
    object_ids.push_back("camera");
    object_ids.push_back("ground");
    planning_scene_interface.removeCollisionObjects(object_ids);

    std::vector<moveit_msgs::CollisionObject> collision_objects;
    //geometry_msgs::PoseStamped current_pose = move_group.getCurrentPose();

    // Add gripper object to robot
    moveit_msgs::AttachedCollisionObject gripper_object;
    gripper_object.link_name = "ee_link";
    gripper_object.object.header.frame_id = "ee_link";
    gripper_object.object.id = "gripper";
    shape_msgs::SolidPrimitive gripper_primitive;
    gripper_primitive.type = gripper_primitive.BOX;
    gripper_primitive.dimensions.resize(3);
    gripper_primitive.dimensions[0] = 0.3;
    gripper_primitive.dimensions[1] = 0.21;
    gripper_primitive.dimensions[2] = 0.08;

    geometry_msgs::Pose gripper_pose;
    gripper_pose.orientation.w = 0.0;
    gripper_pose.position.x = gripper_primitive.dimensions[0]/2;
    gripper_pose.position.y = 0.0;
    gripper_pose.position.z = 0.0;

    gripper_object.object.primitives.push_back(gripper_primitive);
    gripper_object.object.primitive_poses.push_back(gripper_pose);
    gripper_object.object.operation = gripper_object.object.ADD;

    //gripper_object.touch_links = std::vector<std::string>{ "ee_link"};
    
    //planning_scene.world.collision_objects.push_back(gripper_object.object);
    planning_scene.is_diff = true;
    //planning_scene_diff_publisher.publish(planning_scene);

    /* First, define the REMOVE object message*/
    //moveit_msgs::CollisionObject remove_gripper;
    //remove_gripper.id = "gripper";
    //remove_gripper.header.frame_id = "base_link";
    //remove_gripper.operation = remove_gripper.REMOVE;

    /* Carry out the REMOVE + ATTACH operation */
    ROS_INFO("Attaching the gripper to the robot");
    //planning_scene.world.collision_objects.clear();
    //planning_scene.world.collision_objects.push_back(remove_gripper);
    planning_scene.robot_state.attached_collision_objects.push_back(gripper_object);
    //planning_scene_diff_publisher.publish(planning_scene);
    
    // Add camera object to robot
    moveit_msgs::AttachedCollisionObject camera_object;
    camera_object.link_name = "ee_link";
    camera_object.object.header.frame_id = "ee_link";
    camera_object.object.id = "camera";
    shape_msgs::SolidPrimitive camera_primitive;
    camera_primitive.type = camera_primitive.BOX;
    camera_primitive.dimensions.resize(3);
    camera_primitive.dimensions[0] = 0.04;
    camera_primitive.dimensions[1] = 0.09;
    camera_primitive.dimensions[2] = 0.06;

    geometry_msgs::Pose camera_pose;
    camera_pose.orientation.w = 0.0;
    camera_pose.position.x = 0.07;
    camera_pose.position.y = 0.0;
    camera_pose.position.z = (camera_primitive.dimensions[2]+gripper_primitive.dimensions[2])/2;

    camera_object.object.primitives.push_back(camera_primitive);
    camera_object.object.primitive_poses.push_back(camera_pose);
    camera_object.object.operation = camera_object.object.ADD;

    //camera_object.touch_links = std::vector<std::string>{ "ee_link", "gripper"};
    
    //planning_scene.world.collision_objects.push_back(camera_object.object);
    planning_scene.is_diff = true;
    //planning_scene_diff_publisher.publish(planning_scene);

    /* First, define the REMOVE object message*/
    //moveit_msgs::CollisionObject remove_camera;
    //remove_camera.id = "camera";
    //remove_camera.header.frame_id = "base_link";
    //remove_camera.operation = remove_camera.REMOVE;

    /* Carry out the REMOVE + ATTACH operation */
    ROS_INFO("Attaching the camera to the robot");
    //planning_scene.world.collision_objects.clear();
    //planning_scene.world.collision_objects.push_back(remove_camera);
    planning_scene.robot_state.attached_collision_objects.push_back(camera_object);
    planning_scene_diff_publisher.publish(planning_scene);
    //visual_tools.prompt("Press 'next' in the RvizVisualToolsGui window to once the collision object appears in RViz");

    //collision_objects.push_back(camera_object);

    //ROS_INFO_NAMED("tutorial", "Attach the camera to the robot");
    //move_group.attachObject(camera_object.id);

    // Add ground plane object to robot
    moveit_msgs::CollisionObject ground_object;
    ground_object.header.frame_id = move_group.getPlanningFrame();
    ground_object.id = "ground";
    shape_msgs::SolidPrimitive ground_primitive;
    ground_primitive.type = ground_primitive.BOX;
    ground_primitive.dimensions.resize(3);
    ground_primitive.dimensions[0] = 2;
    ground_primitive.dimensions[1] = 2;
    ground_primitive.dimensions[2] = 0.01;

    geometry_msgs::Pose ground_pose;
    ground_pose.orientation.w = 1.0;
    ground_pose.position.x = 0.0;
    ground_pose.position.y = 0.0;
    ground_pose.position.z = -ground_primitive.dimensions[2];

    ground_object.primitives.push_back(ground_primitive);
    ground_object.primitive_poses.push_back(ground_pose);
    ground_object.operation = ground_object.ADD;

    collision_objects.push_back(ground_object);

    ROS_INFO_NAMED("tutorial", "Add ground into the world");
    planning_scene_interface.addCollisionObjects(collision_objects);
    visual_tools.publishText(text_pose, "Add object", rvt::WHITE, rvt::XLARGE);
    visual_tools.trigger();
}

void moveit_robot::basic_cartesian_move(double dist1, double dist2, double dist3){
    move_group.setStartState(*move_group.getCurrentState());
    std::vector<geometry_msgs::Pose> waypoints3;
    geometry_msgs::Pose targetPosition = move_group.getCurrentPose().pose;
    targetPosition.position.x = targetPosition.position.x + dist1;
    targetPosition.position.y = targetPosition.position.y + dist2;
    targetPosition.position.z = targetPosition.position.z + dist3;
    cout << "New Target pose: " << targetPosition << endl;
    waypoints3.push_back(targetPosition);

    moveit_msgs::RobotTrajectory trajectory3;
    const double jump_threshold = 0.0;
    const double eef_step = 0.01;
    double fraction = move_group.computeCartesianPath(waypoints3, eef_step, jump_threshold, trajectory3, true);
    ROS_INFO_NAMED("Robothon", "Visualizing plan - Battery Removing First Step (Cartesian path) (%.2f%% acheived)", fraction * 100.0);

    move_group.execute(trajectory3);
}

void moveit_robot::move_cartesian(double dist1, double dist2, double dist3){

    robot_model_loader::RobotModelLoader robot_model_loader("robot_description");
    robot_model::RobotModelPtr kinematic_model = robot_model_loader.getModel();

    ROS_INFO("Model frame: %s", kinematic_model->getModelFrame().c_str());
   
    robot_state::RobotStatePtr kinematic_state(new robot_state::RobotState(kinematic_model));
    kinematic_state->setToDefaultValues();

    move_group.setStartState(*move_group.getCurrentState());

    geometry_msgs::Pose target_home = move_group.getCurrentPose().pose;

    cout << "Pose:" << move_group.getCurrentPose().pose.position << endl;
    // Vector to store the waypoints for the planning process
    std::vector<geometry_msgs::Pose> waypoints;
    // Stores the first target pose or waypoint
    geometry_msgs::Pose target_pose3 = target_home;
   
    target_pose3.position.x = target_pose3.position.x + dist1;    
    target_pose3.position.y = target_pose3.position.y + dist2;
    target_pose3.position.z = target_pose3.position.z + dist3;

    waypoints.push_back(target_pose3);
    int num_waypoints = plan.trajectory_.joint_trajectory.points.size();  
    cout << "Number of way points :" << num_waypoints << endl;

    moveit_msgs::RobotTrajectory trajectory;
    const double jump_threshold = 0.0;
    const double eef_step = 0.01;
    double fraction = move_group.computeCartesianPath(waypoints, eef_step, jump_threshold, trajectory, true);
    ROS_INFO_NAMED("tutorial", "Visualizing plan 4 (Cartesian path) (%.2f%% acheived)", fraction * 100.0);

    // First to create a RobotTrajectory object
    robot_trajectory::RobotTrajectory rt(move_group.getCurrentState()->getRobotModel(), PLANNING_GROUP);
    // Second get a RobotTrajectory from trajectory
    rt.setRobotTrajectoryMsg(*move_group.getCurrentState(), trajectory);

    // Get RobotTrajectory_msg from RobotTrajectory
    rt.getRobotTrajectoryMsg(trajectory);

    plan.trajectory_ = trajectory;

    const std::vector<std::string> joint_names = plan.trajectory_.joint_trajectory.joint_names;    //gets the names of the joints being updated in the trajectory

    kinematic_state->setVariablePositions(joint_names, plan.trajectory_.joint_trajectory.points.at(0).positions);
   // geometry_msgs::end_effector = Pose;
    const std::string end_effector = move_group.getEndEffectorLink().c_str();

    cout << "End-effector:" << end_effector << endl;
   
    Eigen::Affine3d current_end_effector_state = kinematic_state->getGlobalLinkTransform(end_effector);
    Eigen::Affine3d next_end_effector_state;

    //cout << "current_end_effector_state:" << getGlobalLinkTransform(end_effector)<< endl;

    double euclidean_distance, new_timestamp, old_timestamp, q1, q2, q3, dt1, dt2, v1, v2, a;
    trajectory_msgs::JointTrajectoryPoint *prev_waypoint, *curr_waypoint, *next_waypoint;

     

    for(int i = 0; i < num_waypoints - 1; i++)      //loop through all waypoints
    {
        curr_waypoint = &plan.trajectory_.joint_trajectory.points.at(i);
        next_waypoint = &plan.trajectory_.joint_trajectory.points.at(i+1);
         
        //set joints for next waypoint
        kinematic_state->setVariablePositions(joint_names, next_waypoint->positions);
 
        //do forward kinematics to get cartesian positions of end effector for next waypoint
        next_end_effector_state = kinematic_state->getGlobalLinkTransform(end_effector);
 
        //get euclidean distance between the two waypoints
        euclidean_distance = pow(pow(next_end_effector_state.translation()[0] - current_end_effector_state.translation()[0], 2) +
                            pow(next_end_effector_state.translation()[1] - current_end_effector_state.translation()[1], 2) +
                            pow(next_end_effector_state.translation()[2] - current_end_effector_state.translation()[2], 2), 0.5);
 
        new_timestamp = curr_waypoint->time_from_start.toSec() + (euclidean_distance / 0.05);      //start by printing out all 3 of these!
        old_timestamp = next_waypoint->time_from_start.toSec();

        //update next waypoint timestamp & joint velocities/accelerations if joint velocity/acceleration constraints allow
        if(new_timestamp > old_timestamp)
            next_waypoint->time_from_start.fromSec(new_timestamp);
        else
        {
            //ROS_WARN_NAMED("setAvgCartesianSpeed", "Average speed is too fast. Moving as fast as joint constraints allow.");
        }
         
        //update current_end_effector_state for next iteration
        current_end_effector_state = next_end_effector_state;
    }
     
    //now that timestamps are updated, update joint velocities/accelerations (used updateTrajectory from iterative_time_parameterization as a reference)
    for(int i = 0; i < num_waypoints; i++)
    {
        curr_waypoint = &plan.trajectory_.joint_trajectory.points.at(i);            //set current, previous & next waypoints
        if(i > 0)
            prev_waypoint = &plan.trajectory_.joint_trajectory.points.at(i-1);
        if(i < num_waypoints-1)
            next_waypoint = &plan.trajectory_.joint_trajectory.points.at(i+1);
 
        if(i == 0)          //update dt's based on waypoint (do this outside of loop to save time)
            dt1 = dt2 = next_waypoint->time_from_start.toSec() - curr_waypoint->time_from_start.toSec();
        else if(i < num_waypoints-1)
        {
            dt1 = curr_waypoint->time_from_start.toSec() - prev_waypoint->time_from_start.toSec();
            dt2 = next_waypoint->time_from_start.toSec() - curr_waypoint->time_from_start.toSec();
        }
        else
            dt1 = dt2 = curr_waypoint->time_from_start.toSec() - prev_waypoint->time_from_start.toSec();
 
        for(int j = 0; j < joint_names.size(); j++)     //loop through all joints in waypoint
        {
            if(i == 0)                      //first point
            {
                q1 = next_waypoint->positions.at(j);
                q2 = curr_waypoint->positions.at(j);
                q3 = q1;
            }
            else if(i < num_waypoints-1)    //middle points
            {
                q1 = prev_waypoint->positions.at(j);
                q2 = curr_waypoint->positions.at(j);
                q3 = next_waypoint->positions.at(j);
            }
            else                            //last point
            {
                q1 = prev_waypoint->positions.at(j);
                q2 = curr_waypoint->positions.at(j);
                q3 = q1;
            }
 
            if(dt1 == 0.0 || dt2 == 0.0)
                v1 = v2 = a = 0.0;
            else
            {
                v1 = (q2 - q1)/dt1;
                v2 = (q3 - q2)/dt2;
                a = 2.0*(v2 - v1)/(dt1+dt2);
            }
 
            //actually set the velocity and acceleration
            curr_waypoint->velocities.at(j) = (v1+v2)/2;
            curr_waypoint->accelerations.at(j) = a;
            cout << "curr_waypoint_velocities:" << (v1+v2)/2 << endl; // print velocity valuesc

        }
    }
       
     move_group.asyncExecute(plan);
       
}

bool moveit_robot::plan_to_pose(geometry_msgs::Pose pose){
    bool success = false;

    geometry_msgs::Pose current_pose = move_group.getCurrentPose().pose;
    cout << "Current pose: " << current_pose << endl;
    pose.position.z = current_pose.position.z;
    pose.orientation = current_pose.orientation;
    cout << "Target pose: " << pose << endl;

    pose2plan2.pose = pose;
    pose2plan2.header.seq++;
    pose2plan2.header.stamp = ros::Time::now();
    target_pose_pub.publish(pose2plan2);

    double dist1 = pose.position.x - current_pose.position.x;
    double dist2 = pose.position.y - current_pose.position.y;
    double dist3 = 0;//current_pose.position.z - pose.position.z;
    //basic_cartesian_move(dist1, dist2, dist3);
    //success = true;

    move_group.setPoseTarget(pose);

    success = (move_group.plan(plan) == moveit::planning_interface::MoveItErrorCode::SUCCESS);

    ROS_INFO_NAMED("IK Robot", "Visualizing plan 1 (pose goal) %s", success ? "" : "FAILED");

    // Visualizing plans
    // ^^^^^^^^^^^^^^^^^
    // We can also visualize the plan as a line with markers in RViz.
    ROS_INFO_NAMED("IK Robot", "Visualizing plan 1 as trajectory line");
    visual_tools.publishAxisLabeled(pose, "pose1");
    visual_tools.publishText(text_pose, "Pose Goal", rvt::WHITE, rvt::XLARGE);
    visual_tools.publishTrajectoryLine(plan.trajectory_, joint_model_group);
    visual_tools.trigger();
    if (success){
        visual_tools.prompt("Press 'next' in the RvizVisualToolsGui window to continue the demo");
        move_group.move();
    }
    return success;
}

geometry_msgs::Pose moveit_robot::transform_pose(geometry_msgs::Pose input_pose){
  tf2_ros::Buffer tfBuffer;
  tf2_ros::TransformListener tfListener(tfBuffer);
  geometry_msgs::Pose output_pose1;
  geometry_msgs::Pose output_pose2;
  geometry_msgs::TransformStamped transform1;
  geometry_msgs::TransformStamped transform2;

  while (ros::ok()){
    try{
      transform1 = tfBuffer.lookupTransform("world", "camera_frame",
                                 ros::Time(0));
    
      ROS_INFO("%s", transform1.child_frame_id.c_str());
      ROS_INFO("%f", transform1.transform.translation.x);
      ROS_INFO("%f", transform1.transform.translation.y);
      ROS_INFO("%f", transform1.transform.translation.z);
      ROS_INFO("%f", transform1.transform.rotation.x);
      ROS_INFO("%f", transform1.transform.rotation.y);
      ROS_INFO("%f", transform1.transform.rotation.z);
      ROS_INFO("%f", transform1.transform.rotation.w);

      tf2::doTransform(input_pose, output_pose1, transform1);

      ROS_INFO_STREAM("Input pose: \n" << input_pose);
      ROS_INFO_STREAM("Output pose1: \n" << output_pose1);

      return output_pose1;
    }
    catch (tf2::TransformException &ex) {
      ROS_ERROR("%s",ex.what());
      ros::Duration(1).sleep();
    }
  }
}

void moveit_robot::move_robot(std::map<std::string, double> targetJoints, std::string robot_action, std::string jnt_pos_name){
    
    if( joint_positions.count(jnt_pos_name) )
    {
        targetJoints.clear();
        targetJoints["shoulder_pan_joint"] = joint_positions[jnt_pos_name].angles[0]*3.1416/180;	// (deg*PI/180)
        targetJoints["shoulder_lift_joint"] = joint_positions[jnt_pos_name].angles[1]*3.1416/180;
        targetJoints["elbow_joint"] = joint_positions[jnt_pos_name].angles[2]*3.1416/180;
        targetJoints["wrist_1_joint"] = joint_positions[jnt_pos_name].angles[3]*3.1416/180;
        targetJoints["wrist_2_joint"] = joint_positions[jnt_pos_name].angles[4]*3.1416/180;
        targetJoints["wrist_3_joint"] = joint_positions[jnt_pos_name].angles[5]*3.1416/180;

        robot_status_msg.data = robot_action;
        robot_status_pub.publish(robot_status_msg);

        move_group.setStartState(*move_group.getCurrentState());

        move_group.setJointValueTarget(targetJoints);

        bool success = (move_group.plan(plan) == moveit::planning_interface::MoveItErrorCode::SUCCESS);
        ROS_INFO("Visualizing new move position plan (%.2f%% acheived)",success * 100.0);

        move_group.execute(plan);
    }
    else{
        cout << "Unrecognised joint_position key: " << jnt_pos_name << endl;
    }

}

void moveit_robot::z_move(double dist, double max_velocity_scale_factor){

    //--Cartesian movement planning for straight down movement--//
    // dist is -ve down, +ve up in m
    move_group.setStartState(*move_group.getCurrentState());

    geometry_msgs::Pose target_home = move_group.getCurrentPose().pose;

    geometry_msgs::Pose homeZPosition = move_group.getCurrentPose().pose;

    //move_group.setStartState(*move_group.getCurrentState());

    // Vector to store the waypoints for the planning process
    std::vector<geometry_msgs::Pose> waypoints;
    // Stores the first target pose or waypoint
    geometry_msgs::Pose target_pose3 = target_home;

    target_pose3.position.z = target_pose3.position.z + dist;
    waypoints.push_back(target_pose3);

    // We want the Cartesian path to be interpolated at a resolution of 1 cm
    // which is why we will specify 0.01 as the max step in Cartesian
    // translation.  We will specify the jump threshold as 0.0, effectively disabling it.
    // Warning - disabling the jump threshold while operating real hardware can cause
    // large unpredictable motions of redundant joints and could be a safety issue
    moveit_msgs::RobotTrajectory trajectory;
    const double jump_threshold = 0.0;
    const double eef_step = 0.01;
    double fraction = move_group.computeCartesianPath(waypoints, eef_step, jump_threshold, trajectory, true);
    ROS_INFO_NAMED("tutorial", "Visualizing plan 4 (Cartesian path) (%.2f%% acheived)", fraction * 100.0);

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

    // The trajectory needs to be modified so it will include velocities as well.
    // First to create a RobotTrajectory object
    robot_trajectory::RobotTrajectory rt(move_group.getCurrentState()->getRobotModel(), PLANNING_GROUP);

    // Second get a RobotTrajectory from trajectory
    rt.setRobotTrajectoryMsg(*move_group.getCurrentState(), trajectory);
     
    // Thrid create a IterativeParabolicTimeParameterization object
    trajectory_processing::IterativeParabolicTimeParameterization iptp;
    // Fourth compute computeTimeStamps
    // double max_velocity_scale_factor;
    // if (dist < 0){
    //     max_velocity_scale_factor = 0.05;
    // }
    // else{
    //     max_velocity_scale_factor = 1;
    // }
    bool success = iptp.computeTimeStamps(rt, max_velocity_scale_factor);
    ROS_INFO("Computed time stamp %s",success?"SUCCEDED":"FAILED");
    // Get RobotTrajectory_msg from RobotTrajectory
    rt.getRobotTrajectoryMsg(trajectory);
    // Check trajectory_msg for velocities not empty
    //std::cout << trajectory << std::endl;

    plan.trajectory_ = trajectory;
    ROS_INFO("Visualizing plan 4 (cartesian path) (%.2f%% acheived)",fraction * 100.0);   
    //move_group.execute(plan);

    // You can execute a trajectory like this.
    robot_execute_code = 0;
    ft_srv.request.command_id = ft_srv.request.COMMAND_SET_ZERO;
    if(ft_client.call(ft_srv)){
        ROS_INFO("ret: %s", ft_srv.response.res.c_str());
        ROS_INFO("I heard: FX[%f] FY[%f] FZ[%f] MX[%f] MY[%f] MZ[%f]", ft_readings[0], ft_readings[1], ft_readings[2], ft_readings[3], ft_readings[4], ft_readings[5]);
    }
    std::cout << "Z move dist: " << dist << endl;
    if (dist < 0){
        move_group.asyncExecute(plan);
        double last = 0.0;
        while ((ft_readings[2] > -3) && (robot_execute_code != 1) && ros::ok())
        {
            //if (abs(ft_readings[2]) > abs(last)){
            //    last = ft_readings[2];
            //    std::cout << ft_readings[2] << endl;
            //}
        }
        move_group.stop();
        string execute_result = "unknown";
        if (robot_execute_code == 1){
            execute_result = "Complete";
        }
        else if (robot_execute_code == 0){
            execute_result = "Force Stop";
        }
        std::cout << ">> Robot code: " << robot_execute_code << " (" << execute_result << ")  Force: " << ft_readings[2] << "  Max: " << last << endl;

    }
    else{
        move_group.execute(plan);
    }

}

void home(std::map<std::string, double> &targetJoints, moveit_robot &Robot)
{
    string bring_cmd = "home";
    Robot.move_robot(targetJoints, bring_cmd, bring_cmd);
}


geometry_msgs::Pose look_for_objects(string bring_cmd)
{
    // wait for message received?
    geometry_msgs::Pose object_pose;

    std::string obj2find = bring_cmd.substr(5);
    cout << "Looking for: " << obj2find << endl;
    while(ros::ok()){
        cout << "Looking for: " << obj2find << " detected: " << object_state_msg.Obj_type << endl;
        if (obj2find == object_state_msg.Obj_type){
           object_pose = object_state_msg.Pose;
           return object_pose;
        }
        else{
            ros::Duration(1).sleep();
        }
    }
    // wait for message received?
    //geometry_msgs::Pose object_pose;
    //object_pose.orientation.w = 1.0;
    //object_pose.position.x = 0.05;
    //object_pose.position.y = 0.1;
    //object_pose.position.z = -0.1;
    //return object_pose;
}

bool find_object(string bring_cmd, std::map<std::string, double> &targetJoints, moveit_robot &Robot)
{
    // Move to position above block
    Robot.move_robot(targetJoints, bring_cmd, "look_for_objects");
    ros::Duration(3).sleep(); //Wait to get good image
    bool success = false;
    while (not success and ros::ok()){
        // Look for object
        geometry_msgs::Pose pose_cam_obj = look_for_objects(bring_cmd);
        cout << "Found: " << pose_cam_obj << endl;
        // Transform to world frame
        geometry_msgs::Pose pose_base_obj = Robot.transform_pose(pose_cam_obj);
        // Move to new position above object
        geometry_msgs::Pose target_pose1;
        target_pose1.orientation.x = pose_base_obj.orientation.x;
        target_pose1.orientation.y = pose_base_obj.orientation.y;
        target_pose1.orientation.z = pose_base_obj.orientation.z;
        target_pose1.orientation.w = pose_base_obj.orientation.w;
        target_pose1.position.x = pose_base_obj.position.x;
        target_pose1.position.y = pose_base_obj.position.y;
        target_pose1.position.z = 0.4;
        ROS_INFO_STREAM("Target pose: \n" << target_pose1);
        success = Robot.plan_to_pose(target_pose1);
    }

    if (success){
        cout << "IK success" << endl;
        //Robot.move_group.execute();
        //Robot.move_group.move();

        // Move down, pick block up, move up
        //pick_up_object(Robot, 0.11);

        // Move to stack position
        //Robot.move_robot(targetJoints, bring_cmd, string("final_stack"));

        // Move down, set down block, move up
        //double block_heght = 0.019;
        //double z_move = 0.11 - (stack_height*block_heght);
        //Robot.z_move(-(z_move-block_heght), 0.05);
        //Robot.z_move(-block_heght, 0.01);
        //Robot.open_gripper();
        //Robot.z_move(z_move, 1.0);
    }
    else {
        cout << "Failed to perform IK plan" << endl;
    }
    
    
    return success;
  // Return to home position
    //home(targetJoints, Robot);
}


int main(int argc, char** argv)
{
    // Set up ROS stuff
    string frame_id = "test_ik_planning";
    ros::init(argc, argv, frame_id);
    ros::NodeHandle node_handle;
    ros::AsyncSpinner spinner(1);
    spinner.start();

    ros::Time diag_timeout = ros::Time::now();

    // High level move commands subscriber
	ros::Subscriber subRobotPosition = node_handle.subscribe("RobotMove", 10, robotMoveCallback);

    // Object recognition subscriber
    ros::Subscriber subObjectRecog = node_handle.subscribe("ObjectStates", 10, objectDetectionCallback);

    // Robot object
    moveit_robot Robot(&node_handle);
    // Map to hold values to send to robot
    std::map<std::string, double> targetJoints;

    // Send robot to home position
    home(targetJoints, Robot);

    // wait position
    Robot.robot_status_msg.data = "Done";
    Robot.robot_status_pub.publish(Robot.robot_status_msg);
    cout << ">>>>-- Waiting for command --<<<<" << endl;

    string last_obj_string = "";

    while( ros::ok() )
    {
        targetJoints.clear();

        // Ignore repeat requests
        if (objectString != last_obj_string)
        {
            cout << "Robot Objective: " << objectString << endl;
            last_obj_string = objectString;

            if ( objectString != "")
            {
                Robot.robot_status_msg.data = objectString;
                Robot.robot_status_pub.publish(Robot.robot_status_msg);
                if( objectString.rfind("find_", 0) == 0 )
                { 
                    bool success = find_object(objectString, targetJoints, Robot);
                }
                else if( objectString == "home" )
                {
                    //Robot.robot_status_msg.data = "home";
                    //Robot.robot_status_pub.publish(Robot.robot_status_msg);
                    home(targetJoints, Robot);
                }
                Robot.robot_status_msg.data = "Done";
                Robot.robot_status_pub.publish(Robot.robot_status_msg);
            }
            
            // wait position
            Robot.robot_status_msg.data = "Waiting";
            Robot.robot_status_pub.publish(Robot.robot_status_msg);
            cout << ">>>>-- Waiting for command --<<<<" << endl;
        }
    }

    ros::shutdown();
    return 0;
}
