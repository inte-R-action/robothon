#include <Wire.h>
#include <Adafruit_MPL115A2.h>

#include <ros.h>
#include <std_msgs/Float32.h>


Adafruit_MPL115A2 mpl115a2;

std_msgs::Float32 pressure_msg;

ros::Publisher pub_pressure("pressureRosPort", &pressure_msg);
ros::NodeHandle nh;


float pressureValue = 0.0;

void setup(void) 
{
  Serial.begin(57600);
  mpl115a2.begin();

  nh.initNode();
  nh.advertise(pub_pressure);

}

void loop(void) 
{
  float pressureKPA = 0.0;

  pressureKPA = mpl115a2.getPressure();
  
  pressure_msg.data = pressureKPA;
  pub_pressure.publish(&pressure_msg);

  Serial.println(pressureKPA, 2);

  delay(50);

  nh.spinOnce();

//  delay(100);
}
