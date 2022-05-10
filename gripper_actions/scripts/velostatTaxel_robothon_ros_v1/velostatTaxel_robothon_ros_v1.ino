// author: inte-R-action Lab
// Velostat sensor for Robothon competition

#include <ros.h>
#include <ros/time.h>
#include <std_msgs/String.h>
#include <std_msgs/MultiArrayLayout.h>
#include <std_msgs/MultiArrayDimension.h>
#include <std_msgs/Int32MultiArray.h>
#include <std_msgs/Float32MultiArray.h>

// 3 x 3 matrix sensor
int outCol[3];
int inRow[3];
float frsADC;

float maxValue = 300;
float maxVoltage = 5.0;

// create ROS node
ros::NodeHandle  nh;

std_msgs::Float32MultiArray array_msg_velostat;

// publisher - topic: velostatData
ros::Publisher vibroTactipVelostatPub("velostatData", &array_msg_velostat);

void setup() 
{
  nh.getHardware()->setBaud(57600);  
  nh.initNode();
  nh.advertise(vibroTactipVelostatPub);

  outCol[0] = 8;
  outCol[1] = 9;
  outCol[2] = 10;
  
  inRow[0] = A0;
  inRow[1] = A1;
  inRow[2] = A2;

  for(int i = 0; i < 3; i++)
  {
    pinMode(outCol[i], OUTPUT);
    digitalWrite(outCol[i], LOW);
  }

  // allocate memory for velostat message
  array_msg_velostat.data = (float*)malloc(sizeof(float) * 9);
  array_msg_velostat.data_length = 9;

  for(int i = 0; i < array_msg_velostat.data_length; i++)
    array_msg_velostat.data[i] = 0.0;
}

void loop() 
{

  for( int colNumber = 0; colNumber < 3; colNumber++ )
  {
    readVelostatSensor(colNumber);
    vibroTactipVelostatPub.publish(&array_msg_velostat);
//    delay(1);

//    nh.spinOnce();
//    delay(1);
  }

  nh.spinOnce();
  delay(1);
}

// read contact from velostat sensor
void readVelostatSensor(int colNumber)
{

  for(int i = 0; i < 3; i++)
  {
    if( i == colNumber )
      digitalWrite(outCol[i], HIGH);
    else
      digitalWrite(outCol[i], LOW);
  }
  
//  for(int i = 0; i < array_msg_velostat.data_length; i++)
  for(int i = 0; i < 3; i++)
  {
    frsADC = analogRead(inRow[i]);
    frsADC = frsADC / maxValue * maxVoltage;

    array_msg_velostat.data[(3*colNumber) + i] = frsADC;
  }
}
