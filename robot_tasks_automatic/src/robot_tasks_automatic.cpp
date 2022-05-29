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
#include <robot_tasks/object_state.h>
#include <Eigen/Dense>


#define PI 3.14159265


using namespace std;
using namespace Eigen;


bool gripperActionsReady = false;
bool robotActionsReady = false;

float boxAngle = 0.0;
float blue_button_x;
float blue_button_y;
float red_button_x;
float red_button_y;
float battery_lid_x;
float battery_lid_y;
float coin_battery_x;
float coin_battery_y;
float ethernet_cable_x;
float ethernet_cable_y;
float battery_hole1_x;
float battery_hole1_y;
float battery_hole2_x;
float battery_hole2_y;
float key_x;
float key_y;
float key_lock_x;
float key_lock_y;

float moveTransformX = 0.0;
float moveTransformY = 0.0;

//static const std::string PLANNING_GROUP = "manipulator";
//moveit::planning_interface::MoveGroupInterface move_group(PLANNING_GROUP);
//moveit::planning_interface::PlanningSceneInterface planning_scene_interface;
//const robot_state::JointModelGroup* joint_model_group = move_group.getCurrentState()->getJointModelGroup(PLANNING_GROUP);


void funcTransformPosition(float boxAngleValue, float currentXPos, float currentYPos, float nextXPos, float nextYPos)
{
	MatrixXd translationMatrix(4,4);
	MatrixXd rotationMatrix(4,4);
	//MatrixXd currentLocation(3,1);
	MatrixXd newLocation(4,1);

//    cout << "Angle deg: " << boxAngleValue << endl;

	float boxAngleValueInRad = boxAngleValue * PI / 180.0;

//    cout << "Angle rad: " << boxAngleValueInRad << endl;

	translationMatrix << 1, 0, 0, nextXPos,
						 0, 1, 0,nextYPos,
                         0, 0, 1, 0,   
						 0, 0, 0, 1;


//    cout << "Translation mat: " << translationMatrix << endl;

	//currentLocation << currentXPos, currentYPos, 1;

	//currentLocation = translationMatrix * currentLocation;

	rotationMatrix << cos(boxAngleValueInRad), -sin(boxAngleValueInRad), 0, currentXPos,
				      sin(boxAngleValueInRad),  cos(boxAngleValueInRad), 0, currentYPos,
						 0, 0, 1, 0, 
                         0, 0, 0, 1;

//    cout << "Rotation mat: " << rotationMatrix << endl;
   
	newLocation =  rotationMatrix * translationMatrix;

//    cout << "oUTPUT mat: " << newLocation << endl;

	moveTransformX = newLocation(0,3);
	moveTransformY = newLocation(1,3);
}

// Spiral Search Funtion
const int no_waypoints=20;
struct XY_Array
{
    float arr[no_waypoints][2];
};

/**
 * @brief spiral path= R=a*teta+b, x=R*cos(teta), y=R*sin(teta)
 * 
 * @param sigma_Perror  => standard variation of the position esrror (cm)
 * @param start_point => the point that sprial start from (cm)
 * @return XY_Array  => arr[no_points][2]
 */


XY_Array generate_spiral_path( float sigma_Perror, float start_point[2])
{
  XY_Array waypoints;
  float a=0.05, b=0.02;
  float R_max=4*sigma_Perror;
  float teta_max= (R_max-b)/a;
  float teta_min=0;
  float interval= (teta_max-teta_min)/(no_waypoints-1);
  float teta=0, R=0;
  

  for (int i = 0; i < no_waypoints; i++)
  {
    teta = i*interval;
    R= a*teta+b;
    waypoints.arr[i][0]= R*cos(teta)+start_point[0];
    waypoints.arr[i][1]= R*sin(teta)+start_point[1];
  }
  
 return waypoints;
  
}


// to be completed 
void sprial_force_search(float force_threshold, XY_Array spiral_points, float &X_center, float &Y_center)
{
  float x=0, y=0;
   for (int i = 0; i < no_waypoints; i++)
   {
     x=spiral_points.arr[i][0];
     y=spiral_points.arr[i][1];
   }
   
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
//	boxAngle = msg->data;
    
}

bool dataReceived = false;
void objectDetectionCallback(const robot_tasks::object_state::ConstPtr &msg)
{
    //if (objectString != "")
    //{
      //  try
        //{

        if( dataReceived == false )
        {
//            cout << "Obj_type: " << msg->Obj_type << endl;
            if (msg->Obj_type == "blue_button")
            {
                
                boxAngle = msg->Pose.orientation.z;
                blue_button_x = msg->Pose.position.x;
                blue_button_y = msg->Pose.position.y;
                
//                cout << "Blue button: " << blue_button_x << ", " << blue_button_y << " Angle: " << boxAngle<< endl;
            }


            if (msg->Obj_type == "red_button")
            {
                
                boxAngle = msg->Pose.orientation.z;
                red_button_x = msg->Pose.position.x;
                red_button_y = msg->Pose.position.y;
                
//                cout << "red button: " << red_button_x << ", " << red_button_y << " Angle: " << boxAngle<< endl;
            }


            if (msg->Obj_type == "bholder")
            {
                
                boxAngle = msg->Pose.orientation.z;
                battery_lid_x = msg->Pose.position.x;
                battery_lid_y = msg->Pose.position.y;
                
//                cout << "Battery Lid: " << battery_lid_x << ", " << battery_lid_y << " Angle: " << boxAngle<< endl;
            }

            if (msg->Obj_type == "battery")
            {
                
                boxAngle = msg->Pose.orientation.z;
                coin_battery_x = msg->Pose.position.x;
                coin_battery_y = msg->Pose.position.y;
                
//                cout << "Coin Battery: " << coin_battery_x << ", " << coin_battery_y << " Angle: " << boxAngle<< endl;
            }

            if (msg->Obj_type == "cable")
            {
                
                boxAngle = msg->Pose.orientation.z;
                ethernet_cable_x = msg->Pose.position.x;
                ethernet_cable_y = msg->Pose.position.y;
                
//                cout << "Ethernet Cable: " << ethernet_cable_x << ", " << ethernet_cable_y << " Angle: " << boxAngle<< endl;
            }

            if (msg->Obj_type == "bplace_red")
            {
                
                boxAngle = msg->Pose.orientation.z;
                battery_hole1_x = msg->Pose.position.x;
                battery_hole1_y = msg->Pose.position.y;
                
//                cout << "Red Button Hole: " << battery_hole1_x << ", " << battery_hole1_y << " Angle: " << boxAngle<< endl;
            }

            if (msg->Obj_type == "bplace_blue")
            {
                
                boxAngle = msg->Pose.orientation.z;
                battery_hole2_x = msg->Pose.position.x;
                battery_hole2_y = msg->Pose.position.y;
                
//                cout << "Blue Button Hole: " <<  battery_hole2_x  << ", " <<  battery_hole2_y << " Angle: " << boxAngle<< endl;
            }

            if (msg->Obj_type == "key")
            {
                
                boxAngle = msg->Pose.orientation.z;
                key_x = msg->Pose.position.x;
                key_y = msg->Pose.position.y;
                
//                cout << "Key: " << key_x  << ", " << key_y << " Angle: " << boxAngle<< endl;
            }

            if (msg->Obj_type == "switch")
            {
                
                boxAngle = msg->Pose.orientation.z;
                key_lock_x = msg->Pose.position.x;
                key_lock_y = msg->Pose.position.y;
                
//                cout << "Switch: " << key_lock_x  << ", " << key_lock_y << " Angle: " << boxAngle<< endl;
            }

//            dataReceived = true;
            //MatrixXd currentPosMatrix(6,2);
            //MatrixXd correctePose(6,2);
            //MatrixXd linear_weights(6,6);
            
           //currentPosMatrix<< coin_battery_x  , coin_battery_y,
                             // blue_button_x   , blue_button_y ,
                             // battery_lid_x   , battery_lid_y ,
                             // red_button_x    , red_button_y,
                             // ethernet_cable_x, ethernet_cable_y,
                              //key_x           , key_y;
/*
          linear_weights<< 0.411877239008355,	0.343624400273304,	0.103973097209462,	0.305533460208061,	0.114690629611894,	-0.0456120650932522,
                           0.347921719024269,	0.307997546623599,	0.026901505012907,	0.266764837391930,	0.015477856463131,	-0.140879657858771,
                           0.096838373123642,	0.031596978952468,	0.193490208424825,	0.047768841977478,	0.252824479732135,	 0.273251588241114,
                           0.307325649809278,	0.264378171484973,	0.050159102701001,	0.231880265087672,	0.048940020869170,	-0.0800984690054198,
                           0.145382772389870,	0.062508317242551,	0.238693989103227,	0.079088484157567,	0.310365440626544,	 0.323226876175461,
                          -0.057656307320162,	-0.13363703207185,	0.279367300282538,	-0.08461494339393,	0.376651605111450,	 0.500140203176956;
*/

  /*       
          cout<< "matrix of initial points:"<<endl;
          cout<< currentPosMatrix<<endl;
          cout<<    "***********"   <<endl;
          cout<< "matrix of corrected points:"<<endl;
          cout<< linear_weights*currentPosMatrix<<endl;
*/
/*
          correctePose=linear_weights*currentPosMatrix;
          coin_battery_x = correctePose(0,0);
          coin_battery_y = correctePose(0,1);    
          battery_lid_x = correctePose(2,0);
          battery_lid_y = correctePose(2,1);   
*/ 
         // int a = 0;
        //  cin>> a;                    
                              
                              
        }

/*
            std::string obj2find = objectString.substr(5);
            if (msg->Obj_type == "new_view")
            {
                object_state_msg.Id = msg->Id;
                object_state_msg.Obj_type = msg->Obj_type;
                object_state_msg.Pose = msg->Pose;
                object_state_msg.Header = msg->Header;
            }
            else if (msg->Obj_type == "test")
            {
                object_state_msg.Id = msg->Id;
                object_state_msg.Obj_type = msg->Obj_type;
                object_state_msg.Pose = msg->Pose;
                object_state_msg.Header = msg->Header;
            }
            else if (msg->Obj_type == objectString.substr(5))
            {
                object_state_msg.Id = msg->Id;
                object_state_msg.Obj_type = msg->Obj_type;
                object_state_msg.Pose = msg->Pose;
                object_state_msg.Header = msg->Header;
            }
*/
     //   }
     //   catch (...)
     //   {
//            cout << "Error with: " << msg->Obj_type << endl;
      //  }
    //}
}


void press_blue_button(moveit::planning_interface::MoveGroupInterface *move_group, robot_actions::robotControlParameters robot_actions_msg, gripper_actions::gripperControlParameters gripper_actions_msg, ros::Publisher robot_actions_pub, ros::Publisher gripper_actions_pub)
{
        geometry_msgs::Pose homePosition = move_group->getCurrentPose().pose;

		cout << "robot action: moveToHomePosition" << endl;
        sleep(0.5);
		robot_actions_msg.action = "moveToHomePosition";
		robot_actions_pub.publish(robot_actions_msg);
    	while( !robotActionsReady )
		{
//			cout << "robotActionReady = " << robotActionsReady << endl;
		}	
		robotActionsReady = false;

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
//			cout << "gripperActionReady = " << gripperActionsReady << endl;
		}	
		gripperActionsReady = false;
		sleep(1);

//        int a = 0;
//        cout << "press a key to continue and lock callback function" << endl;
//        cin >> a;
        
    
        homePosition = move_group->getCurrentPose().pose;

//        cout << "Angle box received = " << boxAngle << endl;
//        cout << "press a key to continue" << endl;
//        cin >> a;

        float blueButtonX = blue_button_x;//0.17606; //45deg (-45 gripper) //0.34344; //-90deg (90 gripper) //0.19968; // 90deg (-90 gripper) //0.23359; // 0 deg (0 gripper) //0.219; 90 deg //0.320; 45 degree //180 degree //0.21058;// 90 degree //0.23359; 0 degree
        float blueButtonY = blue_button_y;//0.01354; //45deg (-45 gripper) //0.03180; //-90deg (90 gripper) //-0.05410; // 90deg (-90 gripper) //0.08075; // 0 deg (0 gripper) //0.038; 90 deg // -0.1098;//0.08075;

//        cout << "blueButtonX: "<<blueButtonX << "blueButtonY: "<< blueButtonY<< endl;
//        cout << "press a key to continue" << endl;
//        cin >> a;

       
		cout << "robot action: taskHomePosition" << endl;
        sleep(0.5);
		robot_actions_msg.action = "taskHomePosition";
		robot_actions_msg.robotJoints[5] = boxAngle;    // end-effector angle
		robot_actions_pub.publish(robot_actions_msg);
		while( !robotActionsReady )
		{
//			cout << "robotActionReady = " << robotActionsReady << endl;
		}	
		robotActionsReady = false;

//       cout << "press a key to continue" << endl;
//        cin >> a;
        homePosition = move_group->getCurrentPose().pose;

//        cout << "press a key to continue" << endl;
//        cin >> a;

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
//			cout << "robotActionReady = " << robotActionsReady << endl;
		}	
		robotActionsReady = false;

//       cout << "press a key to continue" << endl;
//        cin >> a;

        float bluebutton_shiftX = -0.01038;
        float bluebutton_shiftY = 0.005; //0.01122 

        homePosition = move_group->getCurrentPose().pose;

//        cout << "original shift X = " << bluebutton_shiftX << endl;
//        cout << "original shift Y = " << bluebutton_shiftY << endl;

        funcTransformPosition(-boxAngle, homePosition.position.x, homePosition.position.y, bluebutton_shiftX, bluebutton_shiftY);

        bluebutton_shiftX = moveTransformX;
        bluebutton_shiftY = moveTransformY;

//  		cout << "Robot Position X =" << homePosition.position.x << endl;
//		cout << "Robot Position Y =" << homePosition.position.y << endl;      
//		cout << "transformed shift X =" << bluebutton_shiftX << endl;
//		cout << "transformed shift Y =" << bluebutton_shiftY << endl;

//       cout << "press a key to continue" << endl;
//        cin >> a;


        // move robot in x and y position on the blue button
		cout << "robot action: shift on x and y" << endl;
        sleep(0.5);
		robot_actions_msg.action = "moveToCartesian";
		robot_actions_msg.position = 0;
		robot_actions_msg.speed = 0;
		robot_actions_msg.force = 0;
		robot_actions_msg.forceDetection = false;
		robot_actions_msg.incrementXaxis = bluebutton_shiftX - homePosition.position.x;
		robot_actions_msg.incrementYaxis = bluebutton_shiftY - homePosition.position.y; 
		robot_actions_msg.incrementZaxis = 0.0;
		robot_actions_pub.publish(robot_actions_msg);
		while( !robotActionsReady )
		{
//			cout << "robotActionReady = " << robotActionsReady << endl;
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
		robot_actions_msg.incrementZaxis = -0.13565;//-0.078;
		robot_actions_pub.publish(robot_actions_msg);
		while( !robotActionsReady )
		{
//			cout << "robotActionReady = " << robotActionsReady << endl;
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
//			cout << "robotActionReady = " << robotActionsReady << endl;
		}	
		robotActionsReady = false;

		// send robot to home position
		cout << "robot action: moveToHomePosition" << endl;
        sleep(0.5);
		robot_actions_msg.action = "moveToHomePosition";
		robot_actions_pub.publish(robot_actions_msg);
		while( !robotActionsReady )
		{
//			cout << "robotActionReady = " << robotActionsReady << endl;
		}	
		robotActionsReady = false;

        homePosition = move_group->getCurrentPose().pose;

//        cout << "Angle box received = " << boxAngle << endl;
//        a = 0;
//        cout << "press a key to continue" << endl;
//        cin >> a;

		cout << "robot action: taskHomePosition" << endl;
        sleep(0.5);
		robot_actions_msg.action = "taskHomePosition";
		robot_actions_msg.robotJoints[5] = boxAngle;    // end-effector angle
		robot_actions_pub.publish(robot_actions_msg);
		while( !robotActionsReady )
		{
//			cout << "robotActionReady = " << robotActionsReady << endl;
		}	
		robotActionsReady = false;
}

void press_red_button(moveit::planning_interface::MoveGroupInterface *move_group, robot_actions::robotControlParameters robot_actions_msg, gripper_actions::gripperControlParameters gripper_actions_msg, ros::Publisher robot_actions_pub, ros::Publisher gripper_actions_pub)
{
        geometry_msgs::Pose homePosition = move_group->getCurrentPose().pose;

		cout << "robot action: moveToHomePosition" << endl;
        sleep(0.5);
		robot_actions_msg.action = "moveToHomePosition";
		robot_actions_pub.publish(robot_actions_msg);
    	while( !robotActionsReady )
		{
//			cout << "robotActionReady = " << robotActionsReady << endl;
		}	
		robotActionsReady = false;

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
//			cout << "gripperActionReady = " << gripperActionsReady << endl;
		}	
		gripperActionsReady = false;
		sleep(1);

//        int a = 0;
//        cout << "press a key to continue and lock callback function" << endl;
//        cin >> a;
        
    
        homePosition = move_group->getCurrentPose().pose;

//        cout << "Angle box received = " << boxAngle << endl;
//        cout << "press a key to continue" << endl;
//        cin >> a;

        float redButtonX = red_button_x;//0.17606; //45deg (-45 gripper) //0.34344; //-90deg (90 gripper) //0.19968; // 90deg (-90 gripper) //0.23359; // 0 deg (0 gripper) //0.219; 90 deg //0.320; 45 degree //180 degree //0.21058;// 90 degree //0.23359; 0 degree
        float redButtonY = red_button_y;//0.01354; //45deg (-45 gripper) //0.03180; //-90deg (90 gripper) //-0.05410; // 90deg (-90 gripper) //0.08075; // 0 deg (0 gripper) //0.038; 90 deg // -0.1098;//0.08075;

//        cout << "redButtonX: "<<redButtonX << "redButtonY: "<< redButtonY<< endl;
//        cout << "press a key to continue" << endl;
//        cin >> a;

       
		cout << "robot action: taskHomePosition" << endl;
        sleep(0.5);
		robot_actions_msg.action = "taskHomePosition";
		robot_actions_msg.robotJoints[5] = boxAngle;    // end-effector angle
		robot_actions_pub.publish(robot_actions_msg);
		while( !robotActionsReady )
		{
//			cout << "robotActionReady = " << robotActionsReady << endl;
		}	
		robotActionsReady = false;

//       cout << "press a key to continue" << endl;
//        cin >> a;
        homePosition = move_group->getCurrentPose().pose;

//        cout << "press a key to continue" << endl;
//        cin >> a;

        // move robot in x and y position on the blue button
		cout << "robot action: move to x and y positions" << endl;
        sleep(0.5);
		robot_actions_msg.action = "moveToCartesian";
		robot_actions_msg.position = 0;
		robot_actions_msg.speed = 0;
		robot_actions_msg.force = 0;
		robot_actions_msg.forceDetection = false;
		robot_actions_msg.incrementXaxis = redButtonX - homePosition.position.x;
		robot_actions_msg.incrementYaxis = redButtonY - homePosition.position.y; 
		robot_actions_msg.incrementZaxis = 0.0;
		robot_actions_pub.publish(robot_actions_msg);
		while( !robotActionsReady )
		{
//			cout << "robotActionReady = " << robotActionsReady << endl;
		}	
		robotActionsReady = false;

//       cout << "press a key to continue" << endl;
//        cin >> a;

        float redbutton_shiftX = -0.01038;
        float redbutton_shiftY = -0.010; //0.01122 

        homePosition = move_group->getCurrentPose().pose;

//        cout << "original shift X = " << redbutton_shiftX << endl;
//        cout << "original shift Y = " << redbutton_shiftY << endl;

        funcTransformPosition(-boxAngle, homePosition.position.x, homePosition.position.y, redbutton_shiftX, redbutton_shiftY);

        redbutton_shiftX = moveTransformX;
        redbutton_shiftY = moveTransformY;

//  		cout << "Robot Position X =" << homePosition.position.x << endl;
//		cout << "Robot Position Y =" << homePosition.position.y << endl;      
//		cout << "transformed shift X =" << redbutton_shiftX << endl;
//		cout << "transformed shift Y =" << redbutton_shiftY << endl;

//       cout << "press a key to continue" << endl;
//        cin >> a;


        // move robot in x and y position on the blue button
		cout << "robot action: shift on x and y" << endl;
        sleep(0.5);
		robot_actions_msg.action = "moveToCartesian";
		robot_actions_msg.position = 0;
		robot_actions_msg.speed = 0;
		robot_actions_msg.force = 0;
		robot_actions_msg.forceDetection = false;
		robot_actions_msg.incrementXaxis = redbutton_shiftX - homePosition.position.x;
		robot_actions_msg.incrementYaxis = redbutton_shiftY - homePosition.position.y; 
		robot_actions_msg.incrementZaxis = 0.0;
		robot_actions_pub.publish(robot_actions_msg);
		while( !robotActionsReady )
		{
//			cout << "robotActionReady = " << robotActionsReady << endl;
		}	
		robotActionsReady = false;

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
//			cout << "robotActionReady = " << robotActionsReady << endl;
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
		robot_actions_msg.incrementZaxis = -0.13565;//-0.078;
		robot_actions_pub.publish(robot_actions_msg);
		while( !robotActionsReady )
		{
//			cout << "robotActionReady = " << robotActionsReady << endl;
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
//			cout << "robotActionReady = " << robotActionsReady << endl;
		}	
		robotActionsReady = false;

		// send robot to home position
		cout << "robot action: moveToHomePosition" << endl;
        sleep(0.5);
		robot_actions_msg.action = "moveToHomePosition";
		robot_actions_pub.publish(robot_actions_msg);
		while( !robotActionsReady )
		{
//			cout << "robotActionReady = " << robotActionsReady << endl;
		}	
		robotActionsReady = false;

        homePosition = move_group->getCurrentPose().pose;

//        cout << "Angle box received = " << boxAngle << endl;
//        a = 0;
//        cout << "press a key to continue" << endl;
//        cin >> a;

		cout << "robot action: taskHomePosition" << endl;
        sleep(0.5);
		robot_actions_msg.action = "taskHomePosition";
		robot_actions_msg.robotJoints[5] = boxAngle;    // end-effector angle
		robot_actions_pub.publish(robot_actions_msg);
		while( !robotActionsReady )
		{
//			cout << "robotActionReady = " << robotActionsReady << endl;
		}	
		robotActionsReady = false;
}

void gripper_orientation(moveit::planning_interface::MoveGroupInterface *move_group, robot_actions::robotControlParameters robot_actions_msg, gripper_actions::gripperControlParameters gripper_actions_msg, ros::Publisher robot_actions_pub, ros::Publisher gripper_actions_pub)
{
		// send robot to home position
		cout << "robot action: moveToHomePosition" << endl;
        sleep(0.5);
		robot_actions_msg.action = "moveToHomePosition";
		robot_actions_pub.publish(robot_actions_msg);
    	while( !robotActionsReady )
		{
//			cout << "robotActionReady = " << robotActionsReady << endl;
		}	
		robotActionsReady = false;
        
        geometry_msgs::Pose homePosition = move_group->getCurrentPose().pose;

//        cout << "Angle box received = " << boxAngle << endl;
//        int a = 0;
//        cout << "press a key to continue" << endl;
//        cin >> a;

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
//			cout << "robotActionReady = " << robotActionsReady << endl;
		}	
		robotActionsReady = false;

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
//			cout << "gripperActionReady = " << gripperActionsReady << endl;
		}	
		gripperActionsReady = false;
		sleep(1);
}


void robot_home(moveit::planning_interface::MoveGroupInterface *move_group, robot_actions::robotControlParameters robot_actions_msg, gripper_actions::gripperControlParameters gripper_actions_msg, ros::Publisher robot_actions_pub, ros::Publisher gripper_actions_pub)
{
		// send robot to home position
		cout << "robot action: moveToHomePosition" << endl;
        sleep(0.5);
		robot_actions_msg.action = "moveToHomePosition";
		robot_actions_pub.publish(robot_actions_msg);
    	while( !robotActionsReady )
		{
//			cout << "robotActionReady = " << robotActionsReady << endl;
		}	
		robotActionsReady = false;
        
        geometry_msgs::Pose homePosition = move_group->getCurrentPose().pose;

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
//			cout << "gripperActionReady = " << gripperActionsReady << endl;
		}	
		gripperActionsReady = false;
		sleep(1);
}

void get_key(moveit::planning_interface::MoveGroupInterface *move_group, robot_actions::robotControlParameters robot_actions_msg, gripper_actions::gripperControlParameters gripper_actions_msg, ros::Publisher robot_actions_pub, ros::Publisher gripper_actions_pub)
{
            geometry_msgs::Pose homePosition = move_group->getCurrentPose().pose;
		    homePosition = move_group->getCurrentPose().pose;
		    cout << "robot action: moveToHomePosition" << endl;
            sleep(0.5);
		    robot_actions_msg.action = "moveToHomePosition";
		    robot_actions_pub.publish(robot_actions_msg);
        	while( !robotActionsReady )
		    {
//			    cout << "robotActionReady = " << robotActionsReady << endl;
		    }	
		    robotActionsReady = false;

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
//			cout << "gripperActionReady = " << gripperActionsReady << endl;
		}	
		gripperActionsReady = false;
		sleep(1);
            
            homePosition = move_group->getCurrentPose().pose;

//            cout << "Angle box received = " << boxAngle << endl;
//            int a = 0;
//            cout << "press a key to continue" << endl;
//            cin >> a;

		    cout << "robot action: taskHomePosition" << endl;
            sleep(0.5);
		    robot_actions_msg.action = "taskHomePosition";
		    robot_actions_msg.robotJoints[5] = boxAngle;    // end-effector angle
		    robot_actions_pub.publish(robot_actions_msg);
		    while( !robotActionsReady )
		    {
//			    cout << "robotActionReady = " << robotActionsReady << endl;
		    }	
		    robotActionsReady = false;


            float keyCenterPosX = key_x;//0.246;
            float keyCenterPosY = key_y;//-0.0650; //-0.0665;
           // geometry_msgs::Pose homePosition = move_group->getCurrentPose().pose;
             homePosition = move_group->getCurrentPose().pose;
            
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
//			    cout << "robotActionReady = " << robotActionsReady << endl;
		    }	
		    robotActionsReady = false;

            //int a = 0;
//            cout << "press to continue" << endl;            
//            cin >> a;

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
//			    cout << "robotActionReady = " << robotActionsReady << endl;
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
//			    cout << "gripperActionReady = " << gripperActionsReady << endl;
		    }	
		    gripperActionsReady = false;
		    sleep(1);
            
//            cout << "press to continue" << endl;            
//            cin >> a;

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
		    robot_actions_msg.incrementZaxis = -0.13565; //-0.078;
		    robot_actions_pub.publish(robot_actions_msg);
		    while( !robotActionsReady )
		    {
//			    cout << "robotActionReady = " << robotActionsReady << endl;
		    }	
		    robotActionsReady = false;

//            cout << "press to continue" << endl;            
//            cin >> a;

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
		    robot_actions_msg.incrementZaxis = 0.0; //0.005;
		    robot_actions_pub.publish(robot_actions_msg);
		    while( !robotActionsReady )
		    {
//			    cout << "robotActionReady = " << robotActionsReady << endl;
		    }	
		    robotActionsReady = false;

//            cout << "press to continue" << endl;            
//            cin >> a;

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
//			    cout << "gripperActionReady = " << gripperActionsReady << endl;
		    }	
		    gripperActionsReady = false;
		    sleep(1);

//            cout << "press to continue" << endl;            
//            cin >> a;

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
//			    cout << "robotActionReady = " << robotActionsReady << endl;
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
//			    cout << "robotActionReady = " << robotActionsReady << endl;
		    }	
		    robotActionsReady = false;

//           cout << "press to continue" << endl;            
//            cin >> a;

        cout << "robot action: taskHomePosition" << endl;
        sleep(0.5);
		robot_actions_msg.action = "moveToJoints";
	    robot_actions_msg.position = 0;
	    robot_actions_msg.speed = 0;
	    robot_actions_msg.force = 0;
	    robot_actions_msg.forceDetection = false;
	    robot_actions_msg.incrementXaxis = 0.0;
	    robot_actions_msg.incrementYaxis = 0.0;
	    robot_actions_msg.incrementZaxis = 0.0;
	    robot_actions_msg.robotJoints[0] = 80;
	    robot_actions_msg.robotJoints[1] = 0;
	    robot_actions_msg.robotJoints[2] = 0;
	    robot_actions_msg.robotJoints[3] = 0;
	    robot_actions_msg.robotJoints[4] =0;
		robot_actions_msg.robotJoints[5] = 0;    // end-effector angle
		robot_actions_pub.publish(robot_actions_msg);
		while( !robotActionsReady )
		{
//			cout << "robotActionReady = " << robotActionsReady << endl;
		}	
		robotActionsReady = false;

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
		    robot_actions_msg.incrementZaxis = -0.080;
		    robot_actions_pub.publish(robot_actions_msg);
		    while( !robotActionsReady )
		    {
//			    cout << "robotActionReady = " << robotActionsReady << endl;
		    }	
		    robotActionsReady = false;

           // move robot to the top of the lid on x and y axis,
		    cout << "robot action: move to release position of objects" << endl;
            sleep(0.5);
		    robot_actions_msg.action = "moveToCartesian";
		    robot_actions_msg.position = 0;
		    robot_actions_msg.speed = 0;
		    robot_actions_msg.force = 0;
		    robot_actions_msg.forceDetection = true;
    		robot_actions_msg.incrementXaxis = 0.0;
    		robot_actions_msg.incrementYaxis = 0.0;
		    robot_actions_msg.incrementZaxis = -0.050;
		    robot_actions_pub.publish(robot_actions_msg);
		    while( !robotActionsReady )
		    {
//			    cout << "robotActionReady = " << robotActionsReady << endl;
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
//			cout << "gripperActionReady = " << gripperActionsReady << endl;
		}	
		gripperActionsReady = false;
		sleep(1);



/*
            geometry_msgs::Pose keytoLock = move_group->getCurrentPose().pose;
            float lockCenterPosX = key_lock_x;//0.3106;
            float lockCenterPosY = key_lock_y;//-0.062;    
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
*/

//           cout << "press to continue" << endl;            
//            cin >> a;

}

void ethernet_port(moveit::planning_interface::MoveGroupInterface *move_group, robot_actions::robotControlParameters robot_actions_msg, gripper_actions::gripperControlParameters gripper_actions_msg, ros::Publisher robot_actions_pub, ros::Publisher gripper_actions_pub)
{
		    // send robot to home position
            geometry_msgs::Pose homePosition = move_group->getCurrentPose().pose;

		    cout << "robot action: moveToHomePosition" << endl;
            sleep(0.5);
		    robot_actions_msg.action = "moveToHomePosition";
		    robot_actions_pub.publish(robot_actions_msg);
        	while( !robotActionsReady )
		    {
//			    cout << "robotActionReady = " << robotActionsReady << endl;
		    }	
		    robotActionsReady = false;
            
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
//			        cout << "gripperActionReady = " << gripperActionsReady << endl;
		        }	
		        gripperActionsReady = false;
		        sleep(1);

          //  homePosition = move_group->getCurrentPose().pose;

//            cout << "Angle box received = " << boxAngle << endl;
//            int a = 0;
//            cout << "press a key to continue" << endl;
//            cin >> a;

		    cout << "robot action: taskHomePosition" << endl;
            sleep(0.5);
		    robot_actions_msg.action = "taskHomePosition";
		    robot_actions_msg.robotJoints[5] = boxAngle;    // end-effector angle
		    robot_actions_pub.publish(robot_actions_msg);
		    while( !robotActionsReady )
		    {
//			    cout << "robotActionReady = " << robotActionsReady << endl;
		    }	
		    robotActionsReady = false;


           homePosition = move_group->getCurrentPose().pose;

//           cout << "press a key to continue" << endl;
//            cin >> a;

///////////////////////////////Reduce the robot on z axis to reach the far point on x axis

           sleep(0.5);
		    robot_actions_msg.action = "moveToJoints";
		    robot_actions_msg.position = 0;
		    robot_actions_msg.speed = 0;
		    robot_actions_msg.force = 0;
		    robot_actions_msg.forceDetection = false;
		    robot_actions_msg.incrementXaxis = 0.0;
		    robot_actions_msg.incrementYaxis = 0.0;
		    robot_actions_msg.incrementZaxis = 0.0;
		    robot_actions_msg.robotJoints[0] = -40.69 + 40.43;
		    robot_actions_msg.robotJoints[1] = -116.69 + 97.68;
		    robot_actions_msg.robotJoints[2] = 79.15 - 33.49;
		    robot_actions_msg.robotJoints[3] = -53.40 + 26.74;
		    robot_actions_msg.robotJoints[4] = -90.70 + 90;
       		robot_actions_msg.robotJoints[5] = 0.0;
		    robot_actions_pub.publish(robot_actions_msg);
		    while( !robotActionsReady )
		    {
//			    cout << "robotActionReady = " << robotActionsReady << endl;
		    }	
		    robotActionsReady = false;

//////////////////////////////////////////////////////////////////////////////////////////////////////


//            float initEthernetPortsPositionX = 0.308; // 0 deg (0 deg gripper) 0.322; // 0 deg (0 deg gripper)
//            float initEthernetPortsPositionY = 0.024; // 0 deg (0 deg gripper) 0.0597; // 0 deg (0 deg gripper)

            float initEthernetPortsPositionX = 0.100; // 0 deg (0 deg gripper) 0.322; // 0 deg (0 deg gripper)
            float initEthernetPortsPositionY = 0.045; // 0 deg (0 deg gripper) 0.0597; // 0 deg (0 deg gripper)
            dataReceived = true;

            homePosition = move_group->getCurrentPose().pose;

            funcTransformPosition(-boxAngle, homePosition.position.x, homePosition.position.y, initEthernetPortsPositionX, initEthernetPortsPositionY);

            initEthernetPortsPositionX = moveTransformX;
            initEthernetPortsPositionY = moveTransformY;   
            
//            cout << "press a key to continue" << endl;
//            cin >> a;

            // move robot 
		    cout << "robot action: move to release position of objects" << endl;
            sleep(0.5);
		    robot_actions_msg.action = "moveToCartesian";
		    robot_actions_msg.position = 0;
		    robot_actions_msg.speed = 0;
		    robot_actions_msg.force = 0;
		    robot_actions_msg.forceDetection = false;
    		robot_actions_msg.incrementXaxis = initEthernetPortsPositionX - homePosition.position.x;
    		robot_actions_msg.incrementYaxis = initEthernetPortsPositionY - homePosition.position.y;
		    robot_actions_msg.incrementZaxis = 0.0;
		    robot_actions_pub.publish(robot_actions_msg);
		    while( !robotActionsReady )
		    {
//			    cout << "robotActionReady = " << robotActionsReady << endl;
		    }	
		    robotActionsReady = false;

            
//            cout << "press to continue" << endl;            
//            cin >> a;

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
//			    cout << "robotActionReady = " << robotActionsReady << endl;
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
//			    cout << "gripperActionReady = " << gripperActionsReady << endl;
		    }	
		    gripperActionsReady = false;
		    sleep(1);
            
//            cout << "press to continue" << endl;            
//            cin >> a;

            homePosition = move_group->getCurrentPose().pose;

//            float ethernetCableX = 0.3138;
//            float ethernetCableY = -0.0205;
//            float targetEthernetPortsPositionX = 0.267;
//            float targetEthernetPortsPositionY = 0.024;
//            float ethernetCableX = 0.267;
//            float ethernetCableY = 0.024;
            float ethernetCableX = ethernet_cable_x; //0.322; // 0 deg (0 deg gripper) 0.308; // 90 deg (-90 deg gripper) 
            float ethernetCableY = ethernet_cable_y;//-0.0597; // 0 deg (0 deg gripper) 0.024; // 90 deg (-90 deg gripper) 

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
//			    cout << "robotActionReady = " << robotActionsReady << endl;
		    }	
		    robotActionsReady = false;

//            cout << "press to continue" << endl;            
//            cin >> a;

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
//			    cout << "robotActionReady = " << robotActionsReady << endl;
		    }	
		    robotActionsReady = false;
            
//            cout << "press to continue" << endl;            
//            cin >> a;

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
//			    cout << "robotActionReady = " << robotActionsReady << endl;
		    }	
		    robotActionsReady = false;

//            cout << "press to continue" << endl;            
//            cin >> a;
          
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
//			    cout << "gripperActionReady = " << gripperActionsReady << endl;
		    }	
		    gripperActionsReady = false;
            sleep(1.0);

//            cout << "press to continue" << endl;            
//            cin >> a;

            geometry_msgs::Pose etherPortReleasePosition = move_group->getCurrentPose().pose;

            float shiftX = 0.003;
            float shiftY = 0.0;

            funcTransformPosition(-boxAngle, etherPortReleasePosition.position.x, etherPortReleasePosition.position.y, shiftX, shiftY);

            shiftX = moveTransformX;
            shiftY = moveTransformY;   
            // move robot on x axis to remove the ethernet cable
		    cout << "robot action: move to release position of objects" << endl;
            sleep(0.5);
		    robot_actions_msg.action = "moveToCartesian";
		    robot_actions_msg.position = 0;
		    robot_actions_msg.speed = 0;
		    robot_actions_msg.force = 0;
		    robot_actions_msg.forceDetection = false;
    		robot_actions_msg.incrementXaxis = shiftX - etherPortReleasePosition.position.x;
    		robot_actions_msg.incrementYaxis = shiftY - etherPortReleasePosition.position.y;
		    robot_actions_msg.incrementZaxis = 0.0;
		    robot_actions_pub.publish(robot_actions_msg);
		    while( !robotActionsReady )
		    {
//			    cout << "robotActionReady = " << robotActionsReady << endl;
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
		    robot_actions_msg.incrementZaxis = 0.050;
		    robot_actions_pub.publish(robot_actions_msg);
		    while( !robotActionsReady )
		    {
//			    cout << "robotActionReady = " << robotActionsReady << endl;
		    }	
		    robotActionsReady = false;
//            cout << "press to continue" << endl;  
//            cin >> a;


        cout << "robot action: taskHomePosition" << endl;
        sleep(0.5);
		robot_actions_msg.action = "moveToJoints";
	    robot_actions_msg.position = 0;
	    robot_actions_msg.speed = 0;
	    robot_actions_msg.force = 0;
	    robot_actions_msg.forceDetection = false;
	    robot_actions_msg.incrementXaxis = 0.0;
	    robot_actions_msg.incrementYaxis = 0.0;
	    robot_actions_msg.incrementZaxis = 0.0;
	    robot_actions_msg.robotJoints[0] = 80;
	    robot_actions_msg.robotJoints[1] = 0;
	    robot_actions_msg.robotJoints[2] = 0;
	    robot_actions_msg.robotJoints[3] = 0;
	    robot_actions_msg.robotJoints[4] =0;
		robot_actions_msg.robotJoints[5] = 0;    // end-effector angle
		robot_actions_pub.publish(robot_actions_msg);
		while( !robotActionsReady )
		{
//			cout << "robotActionReady = " << robotActionsReady << endl;
		}	
		robotActionsReady = false;

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
		    robot_actions_msg.incrementZaxis = -0.080;
		    robot_actions_pub.publish(robot_actions_msg);
		    while( !robotActionsReady )
		    {
//			    cout << "robotActionReady = " << robotActionsReady << endl;
		    }	
		    robotActionsReady = false;

           // move robot to the top of the lid on x and y axis,
		    cout << "robot action: move to release position of objects" << endl;
            sleep(0.5);
		    robot_actions_msg.action = "moveToCartesian";
		    robot_actions_msg.position = 0;
		    robot_actions_msg.speed = 0;
		    robot_actions_msg.force = 0;
		    robot_actions_msg.forceDetection = true;
    		robot_actions_msg.incrementXaxis = 0.0;
    		robot_actions_msg.incrementYaxis = 0.0;
		    robot_actions_msg.incrementZaxis = -0.050;
		    robot_actions_pub.publish(robot_actions_msg);
		    while( !robotActionsReady )
		    {
//			    cout << "robotActionReady = " << robotActionsReady << endl;
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
//			cout << "gripperActionReady = " << gripperActionsReady << endl;
		}	
		gripperActionsReady = false;
		sleep(1);

        cout << "robot action: moveToHomePosition" << endl;
        sleep(0.5);
		robot_actions_msg.action = "moveToHomePosition";
		robot_actions_pub.publish(robot_actions_msg);
		while( !robotActionsReady )
		{
//			cout << "robotActionReady = " << robotActionsReady << endl;
		}	
		robotActionsReady = false;

}

void aa_battery(moveit::planning_interface::MoveGroupInterface *move_group, robot_actions::robotControlParameters robot_actions_msg, gripper_actions::gripperControlParameters gripper_actions_msg, ros::Publisher robot_actions_pub, ros::Publisher gripper_actions_pub)
{

    geometry_msgs::Pose ReleasePosBattery1; 

		// send robot to home position
        dataReceived = false;

		cout << "robot action: moveToHomePosition" << endl;
        sleep(0.5);
		robot_actions_msg.action = "moveToHomePosition";
		robot_actions_pub.publish(robot_actions_msg);
		while( !robotActionsReady )
		{
//			cout << "robotActionReady = " << robotActionsReady << endl;
		}	
		robotActionsReady = false;

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
//			cout << "gripperActionReady = " << gripperActionsReady << endl;
		}	
		gripperActionsReady = false;
		sleep(1);
    
//       cout << "press a key to continue-1" << endl;
//        int a = 0;        
//        cin >> a;

        geometry_msgs::Pose homePosition = move_group->getCurrentPose().pose;
        cout << "robot action: taskHomePosition" << endl;
        sleep(0.5);
		robot_actions_msg.action = "taskHomePosition";
		robot_actions_msg.robotJoints[5] = boxAngle;    // end-effector angle
		robot_actions_pub.publish(robot_actions_msg);
		while( !robotActionsReady )
		{
//			cout << "robotActionReady = " << robotActionsReady << endl;
		}	
		robotActionsReady = false;


//       cout << "press a key to continue-2" << endl;
//        a = 0;        
//        cin >> a;

        dataReceived = true; 

        homePosition = move_group->getCurrentPose().pose;

        float batteryLidX = battery_lid_x;// 0.25142; //0deg (-0 gripper) //0.25257; //45deg (-45 gripper) //0.25536; //-90deg (90 gripper) //0.29150; //90deg (-90 gripper)// 0.2941; // 90 deg 
        float batteryLidY = battery_lid_y;// -0.00873; //0deg (-0 gripper) -0.03342; //45deg (-45 gripper) //0.01705; //-90deg (90 gripper) //-0.03883;//90deg (-90 gripper)//-0.04066; // 90 deg //-0.01245; // 0.01033 is added to move the robot on the lid
        homePosition = move_group->getCurrentPose().pose;

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
//			cout << "robotActionReady = " << robotActionsReady << endl;
		}	
		robotActionsReady = false;
//        cout << "press to continue-3" << endl;
//        cin >> a;

        geometry_msgs::Pose centerLid = move_group->getCurrentPose().pose;
        float batteryLidSlideX = 0; //centerLid.position.x;
        float batteryLidSlideY = 0.01033; // 0.01033 is added to move the robot on the lid

        funcTransformPosition(-boxAngle, centerLid.position.x, centerLid.position.y, batteryLidSlideX, batteryLidSlideY);
        
        batteryLidSlideX = moveTransformX;
        batteryLidSlideY = moveTransformY;
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
//			cout << "robotActionReady = " << robotActionsReady << endl;
		}	
		robotActionsReady = false;


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
		robot_actions_msg.incrementZaxis = -0.13565; //-0.075;
		robot_actions_pub.publish(robot_actions_msg);
		while( !robotActionsReady )
		{
//			cout << "robotActionReady = " << robotActionsReady << endl;
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
		robot_actions_msg.incrementZaxis = -0.0055;
		robot_actions_pub.publish(robot_actions_msg);
		while( !robotActionsReady )
		{
//			cout << "robotActionReady = " << robotActionsReady << endl;
		}	
		robotActionsReady = false;
        

        float LidSlideX = 0.0;
        float LidSlideY = -0.025;
        geometry_msgs::Pose LidSlidePosition = move_group->getCurrentPose().pose;
        funcTransformPosition(-boxAngle, LidSlidePosition.position.x, LidSlidePosition.position.y, LidSlideX, LidSlideY);
        LidSlideX = moveTransformX;
        LidSlideY = moveTransformY;

       // move robot on y axis and slide the lid
		cout << "robot action: slide on lid" << endl;
        sleep(0.5);
		robot_actions_msg.action = "moveToCartesian";
		robot_actions_msg.position = 0;
		robot_actions_msg.speed = 0;
		robot_actions_msg.force = 0;
		robot_actions_msg.forceDetection = false;
		robot_actions_msg.incrementXaxis = LidSlideX - LidSlidePosition.position.x;
		robot_actions_msg.incrementYaxis = LidSlideY - LidSlidePosition.position.y;
		robot_actions_msg.incrementZaxis = 0.0;
		robot_actions_pub.publish(robot_actions_msg);
		while( !robotActionsReady )
		{
//			cout << "robotActionReady = " << robotActionsReady << endl;
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
//			cout << "robotActionReady = " << robotActionsReady << endl;
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
//			cout << "robotActionReady = " << robotActionsReady << endl;
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
//			cout << "gripperActionReady = " << gripperActionsReady << endl;
		}	
		gripperActionsReady = false;
		sleep(1);

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
//			cout << "robotActionReady = " << robotActionsReady << endl;
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
//			cout << "gripperActionReady = " << gripperActionsReady << endl;
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
//			cout << "robotActionReady = " << robotActionsReady << endl;
		}	
		robotActionsReady = false;


        cout << "robot action: taskHomePosition" << endl;
        sleep(0.5);
		robot_actions_msg.action = "moveToJoints";
	    robot_actions_msg.position = 0;
	    robot_actions_msg.speed = 0;
	    robot_actions_msg.force = 0;
	    robot_actions_msg.forceDetection = false;
	    robot_actions_msg.incrementXaxis = 0.0;
	    robot_actions_msg.incrementYaxis = 0.0;
	    robot_actions_msg.incrementZaxis = 0.0;
	    robot_actions_msg.robotJoints[0] = 80;
	    robot_actions_msg.robotJoints[1] = 0;
	    robot_actions_msg.robotJoints[2] = 0;
	    robot_actions_msg.robotJoints[3] = 0;
	    robot_actions_msg.robotJoints[4] = 0;
		robot_actions_msg.robotJoints[5] = 0;    // end-effector angle
		robot_actions_pub.publish(robot_actions_msg);
		while( !robotActionsReady )
		{
//			cout << "robotActionReady = " << robotActionsReady << endl;
		}	
		robotActionsReady = false;

/*
       // move robot in x and y position to leave the lid on the table
		cout << "robot action: move to release (lid) position" << endl;
        sleep(0.5);
		robot_actions_msg.action = "moveToCartesian";
		robot_actions_msg.position = 0;
		robot_actions_msg.speed = 0;
		robot_actions_msg.force = 0;
		robot_actions_msg.forceDetection = false;
		robot_actions_msg.incrementXaxis = -0.100; // -0.100
		robot_actions_msg.incrementYaxis = -0.085; //-0.060
		robot_actions_msg.incrementZaxis = 0.0;
		robot_actions_pub.publish(robot_actions_msg);
		while( !robotActionsReady )
		{
			cout << "robotActionReady = " << robotActionsReady << endl;
		}	
		robotActionsReady = false;
*/
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
		robot_actions_msg.incrementZaxis = -0.080; // -0.090
		robot_actions_pub.publish(robot_actions_msg);
		while( !robotActionsReady )
		{
//			cout << "robotActionReady = " << robotActionsReady << endl;
		}	
		robotActionsReady = false;

		cout << "robot action: move to release (lid) position" << endl;
        sleep(0.5);
		robot_actions_msg.action = "moveToCartesian";
		robot_actions_msg.position = 0;
		robot_actions_msg.speed = 0;
		robot_actions_msg.force = 0;
		robot_actions_msg.forceDetection = true;
		robot_actions_msg.incrementXaxis = 0.0;
		robot_actions_msg.incrementYaxis = 0.0;
		robot_actions_msg.incrementZaxis = -0.050; // -0.090
		robot_actions_pub.publish(robot_actions_msg);
		while( !robotActionsReady )
		{
//			cout << "robotActionReady = " << robotActionsReady << endl;
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
//			cout << "gripperActionReady = " << gripperActionsReady << endl;
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
//			cout << "robotActionReady = " << robotActionsReady << endl;
		}	
		robotActionsReady = false;

//        cout << "Removing battery starts here, press a key" << endl;
//        cin >> a;

    int signBattery=1;
//dataReceived = true;
//      cout << " Box angle before the loop :" << boxAngle << endl;
//    cin >> a;

    for (int batteryNumber=0; batteryNumber<2; batteryNumber++){
		// send robot to home position
		cout << "robot action: moveToHomePosition" << endl;
        sleep(0.5);
		robot_actions_msg.action = "moveToHomePosition";
		robot_actions_pub.publish(robot_actions_msg);
		while( !robotActionsReady )
		{
//			cout << "robotActionReady = " << robotActionsReady << endl;
		}	
		robotActionsReady = false;

//        cout << "home position reached" << endl;

        homePosition = move_group->getCurrentPose().pose;


        cout << "robot action: taskHomePosition" << endl;
        sleep(0.5);
		robot_actions_msg.action = "taskHomePosition";
		robot_actions_msg.robotJoints[5] = boxAngle;// end-effector angle

//        cout << " Box angle inside the loop action1:" << boxAngle << endl;
//        cin >> a;

		robot_actions_pub.publish(robot_actions_msg);
		while( !robotActionsReady )
		{
//			cout << "robotActionReady = " << robotActionsReady << endl;
		}	
		robotActionsReady = false;

//        cout << "gripper rotation reached" << endl;

        homePosition = move_group->getCurrentPose().pose;

//        cout << "home position:" << homePosition<< endl;
//        cout << "centre lid position:" << centerLid << endl;

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
//			cout << "robotActionReady = " << robotActionsReady << endl;
		}	
		robotActionsReady = false;
        
        float battery1SlideX = 0.006 * signBattery;
        float battery1SlideY = 0.008 * signBattery; 

        if( batteryNumber > 0 )
            battery1SlideY = 0.011  * signBattery; //0.008
        else
            battery1SlideY = 0.012 * signBattery; //0.015

        geometry_msgs::Pose BatterySlidePosition = move_group->getCurrentPose().pose;
       	funcTransformPosition(-boxAngle, BatterySlidePosition.position.x, BatterySlidePosition.position.y, battery1SlideX, battery1SlideY);

        battery1SlideX = moveTransformX;
        battery1SlideY = moveTransformY;

    // move robot to the top of the lid on x and y axis,
		cout << "robot action: move to x and y positions on lid" << endl;
        sleep(0.5);
		robot_actions_msg.action = "moveToCartesian";
		robot_actions_msg.position = 0;
		robot_actions_msg.speed = 0;
		robot_actions_msg.force = 0;
		robot_actions_msg.forceDetection = false;
		robot_actions_msg.incrementXaxis =  (battery1SlideX - BatterySlidePosition.position.x);
        robot_actions_msg.incrementYaxis = (battery1SlideY - BatterySlidePosition.position.y);
		robot_actions_msg.incrementZaxis = 0.0;
		robot_actions_pub.publish(robot_actions_msg);
		while( !robotActionsReady )
		{
//			cout << "robotActionReady = " << robotActionsReady << endl;
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
//			cout << "gripperActionReady = " << gripperActionsReady << endl;
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
//			cout << "robotActionReady = " << robotActionsReady << endl;
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
		robot_actions_msg.incrementZaxis = -0.13565; //-0.078;
		robot_actions_pub.publish(robot_actions_msg);
		while( !robotActionsReady )
		{
//			cout << "robotActionReady = " << robotActionsReady << endl;
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
//			cout << "robotActionReady = " << robotActionsReady << endl;
		}	
		robotActionsReady = false;

        float slideBatteryDistanceX = 0.0 * signBattery;
        float slideBatteryDistanceY = -0.008 * signBattery;

        geometry_msgs::Pose slideBatteryDistancePosition = move_group->getCurrentPose().pose;
        funcTransformPosition(-boxAngle, slideBatteryDistancePosition.position.x, slideBatteryDistancePosition.position.y, slideBatteryDistanceX, slideBatteryDistanceY);

        slideBatteryDistanceX = moveTransformX;
        slideBatteryDistanceY = moveTransformY;

       // move robot on y axis to remove the first battery
		cout << "robot action: slide to press battery side" << endl;
        sleep(0.5);
		robot_actions_msg.action = "moveToCartesian";
		robot_actions_msg.position = 0;
		robot_actions_msg.speed = 0;
		robot_actions_msg.force = 0;
		robot_actions_msg.forceDetection = false;
		robot_actions_msg.incrementXaxis =  (slideBatteryDistanceX - slideBatteryDistancePosition.position.x);
        robot_actions_msg.incrementYaxis = (slideBatteryDistanceY - slideBatteryDistancePosition.position.y);
		robot_actions_msg.incrementZaxis = 0.0;
		robot_actions_pub.publish(robot_actions_msg);
		while( !robotActionsReady )
		{
//			cout << "robotActionReady = " << robotActionsReady << endl;
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
//			cout << "robotActionReady = " << robotActionsReady << endl;
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
//			cout << "gripperActionReady = " << gripperActionsReady << endl;
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
//			cout << "robotActionReady = " << robotActionsReady << endl;
		}	
		robotActionsReady = false;

        battery1SlideX = 0.0 * signBattery;
        battery1SlideY = -0.035 * signBattery ;

        if( batteryNumber > 0 )
            battery1SlideY = -0.026 * signBattery;
        else
            battery1SlideY = -0.035 * signBattery;

        BatterySlidePosition = move_group->getCurrentPose().pose;
        funcTransformPosition(-boxAngle, BatterySlidePosition.position.x, BatterySlidePosition.position.y, battery1SlideX, battery1SlideY);

        battery1SlideX = moveTransformX;
        battery1SlideY = moveTransformY;

       // move robot on x and y position to the end of the battery1
		cout << "robot action: move to edge of battery for sliding it" << endl;
        sleep(0.5);
		robot_actions_msg.action = "moveToCartesian";
		robot_actions_msg.position = 0;
		robot_actions_msg.speed = 0;
		robot_actions_msg.force = 0;
		robot_actions_msg.forceDetection = false;
		robot_actions_msg.incrementXaxis =  (battery1SlideX - BatterySlidePosition.position.x);
        robot_actions_msg.incrementYaxis = (battery1SlideY - BatterySlidePosition.position.y);
		robot_actions_msg.incrementZaxis = 0.0;
		robot_actions_pub.publish(robot_actions_msg);
		while( !robotActionsReady )
		{
//			cout << "robotActionReady = " << robotActionsReady << endl;
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
		robot_actions_msg.incrementZaxis = -0.13565; //-0.078;
		robot_actions_pub.publish(robot_actions_msg);
		while( !robotActionsReady )
		{
//			cout << "robotActionReady = " << robotActionsReady << endl;
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
//			    cout << "robotActionReady = " << robotActionsReady << endl;
		    }	
		    robotActionsReady = false;
        }

        battery1SlideX = 0.0 * signBattery;
        battery1SlideY = 0.030 * signBattery;

        if( batteryNumber > 0 )
            battery1SlideY = 0.035 * signBattery;
        else
            battery1SlideY = 0.030 * signBattery; //0.030

        BatterySlidePosition = move_group->getCurrentPose().pose;
        funcTransformPosition(-boxAngle, BatterySlidePosition.position.x, BatterySlidePosition.position.y, battery1SlideX, battery1SlideY);

        battery1SlideX = moveTransformX;
        battery1SlideY = moveTransformY;

       // move robot on y axis to position the battery vertically
		cout << "robot action: slide battery for extraction" << endl;
        sleep(0.5);
		robot_actions_msg.action = "moveToCartesian";
		robot_actions_msg.position = 0;
		robot_actions_msg.speed = 0;
		robot_actions_msg.force = 0;
		robot_actions_msg.forceDetection = false;
		robot_actions_msg.incrementXaxis = 0.0;
   		robot_actions_msg.incrementXaxis =  (battery1SlideX - BatterySlidePosition.position.x);
        robot_actions_msg.incrementYaxis = (battery1SlideY - BatterySlidePosition.position.y);

		robot_actions_msg.incrementZaxis = 0.0;
		robot_actions_pub.publish(robot_actions_msg);
		while( !robotActionsReady )
		{
//			cout << "robotActionReady = " << robotActionsReady << endl;
		}	
		robotActionsReady = false;

        battery1SlideX = 0.0 * signBattery;
        battery1SlideY = -0.010 * signBattery;

        BatterySlidePosition = move_group->getCurrentPose().pose;

        funcTransformPosition(-boxAngle, BatterySlidePosition.position.x, BatterySlidePosition.position.y, battery1SlideX, battery1SlideY);

        battery1SlideX = moveTransformX;
        battery1SlideY = moveTransformY;

       // move robot in 
		cout << "robot action: move robot on y for extraction" << endl;
        sleep(0.5);
		robot_actions_msg.action = "moveToCartesian";
		robot_actions_msg.position = 0;
		robot_actions_msg.speed = 0;
		robot_actions_msg.force = 0;
		robot_actions_msg.forceDetection = false;
		robot_actions_msg.incrementXaxis = battery1SlideX - BatterySlidePosition.position.x;
		robot_actions_msg.incrementYaxis = (battery1SlideY - BatterySlidePosition.position.y);
		robot_actions_msg.incrementZaxis = 0.0;
		robot_actions_pub.publish(robot_actions_msg);
		while( !robotActionsReady )
		{
//			cout << "robotActionReady = " << robotActionsReady << endl;
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
//			cout << "robotActionReady = " << robotActionsReady << endl;
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
//			cout << "gripperActionReady = " << gripperActionsReady << endl;
		}	
		gripperActionsReady = false;
		sleep(1);
//		sleep(0.5);


        battery1SlideX = 0.0 * signBattery;
        battery1SlideY = 0.025 * signBattery;

        BatterySlidePosition = move_group->getCurrentPose().pose;
        funcTransformPosition(-boxAngle, BatterySlidePosition.position.x, BatterySlidePosition.position.y, battery1SlideX, battery1SlideY);

        battery1SlideX = moveTransformX;
        battery1SlideY = moveTransformY;

    // move robot on y axis
		cout << "robot action: move on y to grasp battery" << endl;
        sleep(0.5);
		robot_actions_msg.action = "moveToCartesian";
		robot_actions_msg.position = 0;
		robot_actions_msg.speed = 0;
		robot_actions_msg.force = 0;
		robot_actions_msg.forceDetection = false;
		robot_actions_msg.incrementXaxis = battery1SlideX - BatterySlidePosition.position.x;
		robot_actions_msg.incrementYaxis = (battery1SlideY - BatterySlidePosition.position.y);
		robot_actions_msg.incrementZaxis = 0.0;
		robot_actions_pub.publish(robot_actions_msg);
		while( !robotActionsReady )
		{
//			cout << "robotActionReady = " << robotActionsReady << endl;
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
            robot_actions_msg.incrementZaxis = -0.035;

		robot_actions_pub.publish(robot_actions_msg);
		while( !robotActionsReady )
		{
//			cout << "robotActionReady = " << robotActionsReady << endl;
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
//			    cout << "gripperActionReady = " << gripperActionsReady << endl;
		    }	
		    gripperActionsReady = false;
		    sleep(1);
        }

       // move robot down
        if( batteryNumber > 0 )
        {

            battery1SlideX = 0.0;
            battery1SlideY = 0.030; 

            BatterySlidePosition = move_group->getCurrentPose().pose;
            funcTransformPosition(-boxAngle, BatterySlidePosition.position.x, BatterySlidePosition.position.y, battery1SlideX, battery1SlideY);

            battery1SlideX = moveTransformX;
            battery1SlideY = moveTransformY;

		    cout << "robot action: move down to detect second battery for sliding it with no force" << endl;
            sleep(0.5);
		    robot_actions_msg.action = "moveToCartesian";
		    robot_actions_msg.position = 0;
		    robot_actions_msg.speed = 0;
		    robot_actions_msg.force = 0;
		    robot_actions_msg.forceDetection = false;
    		robot_actions_msg.incrementXaxis = battery1SlideX - BatterySlidePosition.position.x;
    		robot_actions_msg.incrementYaxis = battery1SlideY - BatterySlidePosition.position.y;
		    robot_actions_msg.incrementZaxis = 0.0;
		    robot_actions_pub.publish(robot_actions_msg);
		    while( !robotActionsReady )
		    {
//			    cout << "robotActionReady = " << robotActionsReady << endl;
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
//			cout << "gripperActionReady = " << gripperActionsReady << endl;
		}	
		gripperActionsReady = false;
		sleep(1);

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
//			cout << "robotActionReady = " << robotActionsReady << endl;
		}	
		robotActionsReady = false;


//        cout << "Here starts movement to place battery in hole" << endl;
//        cout << "press a key" << endl;
//        a = 0;
//        cin >> a;

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
		robot_actions_msg.incrementZaxis = 0.020;
		robot_actions_pub.publish(robot_actions_msg);
		while( !robotActionsReady )
		{
//			cout << "robotActionReady = " << robotActionsReady << endl;
		}	
		robotActionsReady = false;  

        cout << "robot action: taskHomePosition" << endl;
        sleep(0.5);
		robot_actions_msg.action = "moveToJoints";
	    robot_actions_msg.position = 0;
	    robot_actions_msg.speed = 0;
	    robot_actions_msg.force = 0;
	    robot_actions_msg.forceDetection = false;
	    robot_actions_msg.incrementXaxis = 0.0;
	    robot_actions_msg.incrementYaxis = 0.0;
	    robot_actions_msg.incrementZaxis = 0.0;
	    robot_actions_msg.robotJoints[0] = 80;
	    robot_actions_msg.robotJoints[1] = 0;
	    robot_actions_msg.robotJoints[2] = 0;
	    robot_actions_msg.robotJoints[3] = 0;
	    robot_actions_msg.robotJoints[4] = 0;
		robot_actions_msg.robotJoints[5] = 0;    // end-effector angle
		robot_actions_pub.publish(robot_actions_msg);
		while( !robotActionsReady )
		{
//			cout << "robotActionReady = " << robotActionsReady << endl;
		}	
		robotActionsReady = false;

    // move robot on z axis down to the battery hole
		cout << "robot action: move robot up" << endl;
        sleep(0.5);
		robot_actions_msg.action = "moveToCartesian";
		robot_actions_msg.position = 0;
		robot_actions_msg.speed = 0;
		robot_actions_msg.force = 0;
		robot_actions_msg.forceDetection = true;
		robot_actions_msg.incrementXaxis = 0.0;
		robot_actions_msg.incrementYaxis = 0.0;
		robot_actions_msg.incrementZaxis = -0.080;
		robot_actions_pub.publish(robot_actions_msg);
		while( !robotActionsReady )
		{
//			cout << "robotActionReady = " << robotActionsReady << endl;
		}	
		robotActionsReady = false;  

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
		robot_actions_msg.incrementZaxis = -0.050;
		robot_actions_pub.publish(robot_actions_msg);
		while( !robotActionsReady )
		{
//			cout << "robotActionReady = " << robotActionsReady << endl;
		}	
		robotActionsReady = false;  

//cin >> a;

                
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
//			cout << "gripperActionReady = " << gripperActionsReady << endl;
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
//			cout << "robotActionReady = " << robotActionsReady << endl;
		}	
		robotActionsReady = false;
        signBattery = signBattery * -1;

    }
}


void coin_battery(moveit::planning_interface::MoveGroupInterface *move_group, robot_actions::robotControlParameters robot_actions_msg, gripper_actions::gripperControlParameters gripper_actions_msg, ros::Publisher robot_actions_pub, ros::Publisher gripper_actions_pub)
{
            geometry_msgs::Pose homePosition = move_group->getCurrentPose().pose;

           // close gripper to sepcific distance to extract batteries
		    cout << "gripper action: close gripper to release battery" << endl;
		    gripper_actions_msg.action = "close";
		    gripper_actions_msg.position = 220;
		    gripper_actions_msg.speed = 50;
		    gripper_actions_msg.force = 50;
		    gripper_actions_msg.useContactDetection = true;
		    gripper_actions_pub.publish(gripper_actions_msg);
		    while( !gripperActionsReady )
		    {
//			    cout << "gripperActionReady = " << gripperActionsReady << endl;
		    }	
		    gripperActionsReady = false;
		    sleep(1);

            homePosition = move_group->getCurrentPose().pose;

		    cout << "robot action: moveToHomePosition" << endl;
            sleep(0.5);
		    robot_actions_msg.action = "moveToHomePosition";
		    robot_actions_pub.publish(robot_actions_msg);
        	while( !robotActionsReady )
		    {
//			    cout << "robotActionReady = " << robotActionsReady << endl;
		    }	
		    robotActionsReady = false;
            
            homePosition = move_group->getCurrentPose().pose;

//            cout << "Angle box received = " << boxAngle << endl;
//            int a = 0;
//            cout << "press a key to continue" << endl;
//            cin >> a;

		    cout << "robot action: taskHomePosition" << endl;
            sleep(0.5);
		    robot_actions_msg.action = "taskHomePosition";
		    robot_actions_msg.robotJoints[5] = boxAngle + 90.0;    // end-effector angle
		    robot_actions_pub.publish(robot_actions_msg);
		    while( !robotActionsReady )
		    {
//			    cout << "robotActionReady = " << robotActionsReady << endl;
		    }	
		    robotActionsReady = false;

//           cout << "press a key to continue" << endl;
//            cin >> a;

//            a = 0;
//            cout << "press to continue" << endl;
//            cin >> a;


            homePosition = move_group->getCurrentPose().pose;

            float coinBatteryX = coin_battery_x; //0.3269; //-90 deg (90 gripper) 0.2440; //90 deg (-90 gripper) 0.3248; // 0 degree 
            float coinBatteryY = coin_battery_y; //-0.07712; //-90 deg (90 gripper) 0.02943; // 90 deg (-90 gripper)  0.08600; //0 degree 

            dataReceived = true;
///////////////////////////////Reduce the robot on z axis to reach the far point on x axis


           sleep(0.5);
		    robot_actions_msg.action = "moveToJoints";
		    robot_actions_msg.position = 0;
		    robot_actions_msg.speed = 0;
		    robot_actions_msg.force = 0;
		    robot_actions_msg.forceDetection = false;
		    robot_actions_msg.incrementXaxis = 0.0;
		    robot_actions_msg.incrementYaxis = 0.0;
		    robot_actions_msg.incrementZaxis = 0.0;
		    robot_actions_msg.robotJoints[0] = -40.69 + 40.43;
		    robot_actions_msg.robotJoints[1] = -116.69 + 97.68;
		    robot_actions_msg.robotJoints[2] = 79.15 - 33.49;
		    robot_actions_msg.robotJoints[3] = -53.40 + 26.74;
		    robot_actions_msg.robotJoints[4] = -90.70 + 90;
       		robot_actions_msg.robotJoints[5] = 0.0;
		    robot_actions_pub.publish(robot_actions_msg);
		    while( !robotActionsReady )
		    {
//			    cout << "robotActionReady = " << robotActionsReady << endl;
		    }	
		    robotActionsReady = false;


//////////////////////////////////////////////////////////////////////////////////////////////////////
        // move robot to the top of the lid on x and y axis,
		    cout << "robot action: move to x and y to top of coin battery" << endl;
            sleep(0.5);
		    robot_actions_msg.action = "moveToCartesian";
		    robot_actions_msg.position = 0;
		    robot_actions_msg.speed = 0;
		    robot_actions_msg.force = 0;
		    robot_actions_msg.forceDetection = false;
		    robot_actions_msg.incrementXaxis = coinBatteryX - homePosition.position.x; //- 0.010);
		    robot_actions_msg.incrementYaxis = coinBatteryY - homePosition.position.y;
		    robot_actions_msg.incrementZaxis = 0.0;
		    robot_actions_pub.publish(robot_actions_msg);
		    while( !robotActionsReady )
		    {
//			    cout << "robotActionReady = " << robotActionsReady << endl;
		    }	
		    robotActionsReady = false;

//            cout << "press to continue" << endl;
//            cin >> a;

            geometry_msgs::Pose CoinBatteryReleaseMove = move_group->getCurrentPose().pose;

            float shiftCoinBatX = 0.0080; //0.0025;
            float shiftCoinBatY = 0.0;

            funcTransformPosition(-boxAngle, CoinBatteryReleaseMove.position.x, CoinBatteryReleaseMove.position.y, shiftCoinBatX, shiftCoinBatY);

            shiftCoinBatX = moveTransformX;
            shiftCoinBatY = moveTransformY;            
            
           // move robot to the top of the coin battery on x and y axis,
		    cout << "robot action: move on y direction to release coin battery" << endl;
            sleep(0.5);
		    robot_actions_msg.action = "moveToCartesian";
		    robot_actions_msg.position = 0;
		    robot_actions_msg.speed = 0;
		    robot_actions_msg.force = 0;
		    robot_actions_msg.forceDetection = false;
		    robot_actions_msg.incrementXaxis = shiftCoinBatX - CoinBatteryReleaseMove.position.x;
		    robot_actions_msg.incrementYaxis = shiftCoinBatY - CoinBatteryReleaseMove.position.y;
		    robot_actions_msg.incrementZaxis = 0.0;
		    robot_actions_pub.publish(robot_actions_msg);
		    while( !robotActionsReady )
		    {
//			    cout << "robotActionReady = " << robotActionsReady << endl;
		    }	
		    robotActionsReady = false;            

            //cout << "center Position_x = " << (coinBatteryX - homePosition.position.x + 0.007) << ", " << coinBatteryX - homePosition.position.x<< endl;    
//            cout << "press to continue" << endl;
//            cin >> a;

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
		    robot_actions_msg.incrementZaxis = -0.078; // -0.078
		    robot_actions_pub.publish(robot_actions_msg);
		    while( !robotActionsReady )
		    {
//			    cout << "robotActionReady = " << robotActionsReady << endl;
		    }	
		    robotActionsReady = false;

//            cout << "press to continue" << endl;
//            cin >> a;

/*
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
		    robot_actions_msg.incrementZaxis = 0.020; //0.020;
		    robot_actions_pub.publish(robot_actions_msg);
		    while( !robotActionsReady )
		    {
//			    cout << "robotActionReady = " << robotActionsReady << endl;
		    }	
		    robotActionsReady = false;
*/
            geometry_msgs::Pose CoinBatteryPosition = move_group->getCurrentPose().pose;

            float displacementCoinPositionX = 0.0;
            float displacementCoinPositionY = 0.015; //0.024; //0.0340

            funcTransformPosition(-boxAngle, CoinBatteryPosition.position.x, CoinBatteryPosition.position.y, displacementCoinPositionX, displacementCoinPositionY);

            displacementCoinPositionX = moveTransformX;
            displacementCoinPositionY = moveTransformY;

           // move robot to the top of the lid on x and y axis,
		    cout << "robot action: move down to detect coin battery without force detection" << endl;
            sleep(0.5);
		    robot_actions_msg.action = "moveToCartesian";
		    robot_actions_msg.position = 0;
		    robot_actions_msg.speed = 0;
		    robot_actions_msg.force = 0;
		    robot_actions_msg.forceDetection = false;
//            robot_actions_msg.incrementXaxis = 0.0;
//		    robot_actions_msg.incrementYaxis = displacementCoinPositionY - CoinBatteryPosition.position.y;
		    robot_actions_msg.incrementXaxis = displacementCoinPositionX - CoinBatteryPosition.position.x;
		    robot_actions_msg.incrementYaxis = displacementCoinPositionY - CoinBatteryPosition.position.y;
		    robot_actions_msg.incrementZaxis = 0.0;
		    robot_actions_pub.publish(robot_actions_msg);
		    while( !robotActionsReady )
		    {
//			    cout << "robotActionReady = " << robotActionsReady << endl;
		    }	
		    robotActionsReady = false;

//            cout << "press to continue" << endl;
//            cin >> a;

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
		    robot_actions_msg.incrementZaxis = -0.003; //-0.023; //-0.030;
		    robot_actions_pub.publish(robot_actions_msg);
		    while( !robotActionsReady )
		    {
//			    cout << "robotActionReady = " << robotActionsReady << endl;
		    }	
		    robotActionsReady = false;

//            cout << "press to continue" << endl;
//            cin >> a;

            geometry_msgs::Pose CoinBatteryReleaseAction = move_group->getCurrentPose().pose;

            float shiftX = 0.0;
            float shiftY = -0.060; //-0.040;

            funcTransformPosition(-boxAngle, CoinBatteryReleaseAction.position.x, CoinBatteryReleaseAction.position.y, shiftX, shiftY);

            shiftX = moveTransformX;
            shiftY = moveTransformY;            
            
           // move robot to the top of the coin battery on x and y axis,
		    cout << "robot action: move on y direction to release coin battery" << endl;
            sleep(0.5);
		    robot_actions_msg.action = "moveToCartesian";
		    robot_actions_msg.position = 0;
		    robot_actions_msg.speed = 0;
		    robot_actions_msg.force = 0;
		    robot_actions_msg.forceDetection = false;
		    //robot_actions_msg.incrementXaxis = 0.0;
		   // robot_actions_msg.incrementYaxis = -0.018;
		    robot_actions_msg.incrementXaxis = shiftX - CoinBatteryReleaseAction.position.x;
		    robot_actions_msg.incrementYaxis = shiftY - CoinBatteryReleaseAction.position.y;
		    robot_actions_msg.incrementZaxis = 0.0;
		    robot_actions_pub.publish(robot_actions_msg);
		    while( !robotActionsReady )
		    {
//			    cout << "robotActionReady = " << robotActionsReady << endl;
		    }	
		    robotActionsReady = false;

//            cout << "press to continue" << endl;
//            cin >> a;

            geometry_msgs::Pose releasePositionOfCoinBattery = move_group->getCurrentPose().pose;

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
//			    cout << "robotActionReady = " << robotActionsReady << endl;
		    }	
		    robotActionsReady = false;

//            cout << "press to continue" << endl;
//            cin >> a;
/*
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
*/
           // move robot to 

     // open gripper to sepcific distance to extract batteries
		    cout << "gripper action: open gripper to release battery" << endl;
		    gripper_actions_msg.action = "open";
		    gripper_actions_msg.position = 170;
		    gripper_actions_msg.speed = 50;
		    gripper_actions_msg.force = 200;
		    gripper_actions_msg.useContactDetection = true;
		    gripper_actions_pub.publish(gripper_actions_msg);
		    while( !gripperActionsReady )
		    {
//			    cout << "gripperActionReady = " << gripperActionsReady << endl;
		    }	
		    gripperActionsReady = false;
		    sleep(1);


		    cout << "robot action: move down to grasp coin battery" << endl;
            sleep(0.5);
		    robot_actions_msg.action = "moveToCartesian";
		    robot_actions_msg.position = 0;
		    robot_actions_msg.speed = 0;
		    robot_actions_msg.force = 0;
		    robot_actions_msg.forceDetection = true;
		    robot_actions_msg.incrementXaxis = 0.0;
		    robot_actions_msg.incrementYaxis = 0.0;
		    robot_actions_msg.incrementZaxis = -0.055;
		    robot_actions_pub.publish(robot_actions_msg);
		    while( !robotActionsReady )
		    {
//			    cout << "robotActionReady = " << robotActionsReady << endl;
		    }	
		    robotActionsReady = false;

//            cout << "press to continue" << endl;
//            cin >> a;

     // open gripper to sepcific distance to extract batteries
		    cout << "gripper action: open gripper to release battery" << endl;
		    gripper_actions_msg.action = "open";
		    gripper_actions_msg.position = 230;
		    gripper_actions_msg.speed = 50;
		    gripper_actions_msg.force = 200;
		    gripper_actions_msg.useContactDetection = true;
		    gripper_actions_pub.publish(gripper_actions_msg);
		    while( !gripperActionsReady )
		    {
//			    cout << "gripperActionReady = " << gripperActionsReady << endl;
		    }	
		    gripperActionsReady = false;
		    sleep(1);

//            cout << "press to continue" << endl;
//            cin >> a;

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
//			    cout << "robotActionReady = " << robotActionsReady << endl;
		    }	
		    robotActionsReady = false;

//            cout << "press to continue" << endl;
//            cin >> a;

        cout << "robot action: taskHomePosition" << endl;
        sleep(0.5);
		robot_actions_msg.action = "moveToJoints";
	    robot_actions_msg.position = 0;
	    robot_actions_msg.speed = 0;
	    robot_actions_msg.force = 0;
	    robot_actions_msg.forceDetection = false;
	    robot_actions_msg.incrementXaxis = 0.0;
	    robot_actions_msg.incrementYaxis = 0.0;
	    robot_actions_msg.incrementZaxis = 0.0;
	    robot_actions_msg.robotJoints[0] = 80;
	    robot_actions_msg.robotJoints[1] = 0;
	    robot_actions_msg.robotJoints[2] = 0;
	    robot_actions_msg.robotJoints[3] = 0;
	    robot_actions_msg.robotJoints[4] =0;
		robot_actions_msg.robotJoints[5] = 0;    // end-effector angle
		robot_actions_pub.publish(robot_actions_msg);
		while( !robotActionsReady )
		{
//			cout << "robotActionReady = " << robotActionsReady << endl;
		}	
		robotActionsReady = false;

//            cout << "press to continue" << endl;
//            cin >> a;

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
		    robot_actions_msg.incrementZaxis = -0.080;
		    robot_actions_pub.publish(robot_actions_msg);
		    while( !robotActionsReady )
		    {
//			    cout << "robotActionReady = " << robotActionsReady << endl;
		    }	
		    robotActionsReady = false;

           // move robot to the top of the lid on x and y axis,
		    cout << "robot action: move to release position of objects" << endl;
            sleep(0.5);
		    robot_actions_msg.action = "moveToCartesian";
		    robot_actions_msg.position = 0;
		    robot_actions_msg.speed = 0;
		    robot_actions_msg.force = 0;
		    robot_actions_msg.forceDetection = true;
    		robot_actions_msg.incrementXaxis = 0.0;
    		robot_actions_msg.incrementYaxis = 0.0;
		    robot_actions_msg.incrementZaxis = -0.050;
		    robot_actions_pub.publish(robot_actions_msg);
		    while( !robotActionsReady )
		    {
//			    cout << "robotActionReady = " << robotActionsReady << endl;
		    }	
		    robotActionsReady = false;

//            cout << "press to continue" << endl;
//            cin >> a;

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
//			    cout << "gripperActionReady = " << gripperActionsReady << endl;
		    }	
		    gripperActionsReady = false;
		    sleep(1);

//            cout << "press to continue" << endl;
//            cin >> a;

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
//			    cout << "robotActionReady = " << robotActionsReady << endl;
		    }	
		    robotActionsReady = false;

//            cout << "press to continue" << endl;
//            cin >> a;

		    cout << "robot action: moveToHomePosition" << endl;
            sleep(0.5);
		    robot_actions_msg.action = "moveToHomePosition";
		    robot_actions_pub.publish(robot_actions_msg);
		    while( !robotActionsReady )
		    {
//			    cout << "robotActionReady = " << robotActionsReady << endl;
		    }	
		    robotActionsReady = false;

}


int main(int argc, char** argv)
{
    ros::init(argc, argv, "robot_tasks_automatic");

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
    ros::Subscriber subObjectRecog = node_handle.subscribe("ObjectStates", 10, objectDetectionCallback);


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


    int nTasks = 7;
    int nTasksIterations = 0;

    int taskOrder[nTasks];

    for(int i = 0; i < nTasks; i++)
    {
        taskOrder[i] = 0;
    }

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
//			cout << "gripperActionReady = " << gripperActionsReady << endl;
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
		{        /****************************************************/

        }

        cout << "**********************************" << endl;
        cout << "task numbers" << endl;
        cout << "1 - align end-effector to box" << endl;
        cout << "2 - blue button" << endl;
        cout << "3 - key " << endl;
        cout << "4 - ethernet cable 2" << endl;
        cout << "5 - AA batteries" << endl;
        cout << "6 - coin battery" << endl;
        cout << "7 - red button" << endl;
		cout << "8 - home position" << endl;
		cout << "9 - exit" << endl;
        cout << "**********************************" << endl;


        cout << "Input iterations: " << endl;
        cin >> nTasksIterations;

        cout << "Input number of tasks: " << endl;
        cin >> nTasks;
   
        cout << "Input the sequence of tasks: " << endl;
        for( int i = 0; i < nTasks; i++ )
            cin >> taskOrder[i];

        cout << "Task ordered entered: ";
        for( int i = 0; i < nTasks; i++ )
            cout << taskOrder[i] << " ";

        cout << "Starting in 5 sec...";
        sleep(5);

        for( int j = 0; j < nTasksIterations; j++)
        {
            cout << "Task iteration: " << j << endl;

            for( int i = 0; i < nTasks; i++ )
            {
                if( taskOrder[i] == 1 )
                {
                    cout << "task order 1 - gripper orientation" << endl;
                    gripper_orientation(&move_group, robot_actions_msg, gripper_actions_msg, robot_actions_pub, gripper_actions_pub);
                    cout << "task 1 completed" << endl;
                }
                else if( taskOrder[i] == 2 )
                {
                    cout << "task order 2 - blue button" << endl;
                    press_blue_button(&move_group, robot_actions_msg, gripper_actions_msg, robot_actions_pub, gripper_actions_pub);
                    cout << "task 2 completed" << endl;
                }
                else if( taskOrder[i] == 3 )
                {
                    cout << "task order 3 - get key" << endl;
                    get_key(&move_group, robot_actions_msg, gripper_actions_msg, robot_actions_pub, gripper_actions_pub);
                    cout << "task 3 completed" << endl;
                }
                else if( taskOrder[i] == 4 )
                {
                    cout << "task order 4 - ethernet port" << endl;
                    ethernet_port(&move_group, robot_actions_msg, gripper_actions_msg, robot_actions_pub, gripper_actions_pub);
                    cout << "task 4 completed" << endl;
                }
                else if( taskOrder[i] == 5 )
                {
                    cout << "task order 5 - AA battery" << endl;
                    aa_battery(&move_group, robot_actions_msg, gripper_actions_msg, robot_actions_pub, gripper_actions_pub);
                    cout << "task 5 completed" << endl;
                }
                else if( taskOrder[i] == 6 )
                {
                    cout << "task order 6 - coin battery" << endl;
                    coin_battery(&move_group, robot_actions_msg, gripper_actions_msg, robot_actions_pub, gripper_actions_pub);
                    cout << "task 6 completed" << endl;
                }
                else if( taskOrder[i] == 7 )
                {
                    cout << "task order 7 - red button" << endl;
                    press_red_button(&move_group, robot_actions_msg, gripper_actions_msg, robot_actions_pub, gripper_actions_pub);
                    cout << "task 2 completed" << endl;
                }
                if( taskOrder[i] == 8 )
                {
                    cout << "task order 8 - home position" << endl;
                    robot_home(&move_group, robot_actions_msg, gripper_actions_msg, robot_actions_pub, gripper_actions_pub);
                    cout << "task 1 completed" << endl;
                }
                else
                {
                    cout << "nothing selected" << endl;
                }
                
            }
        }
    }

    usleep(1000);
    ros::waitForShutdown();

    return 0;
}
