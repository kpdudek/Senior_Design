#include <ros.h>
//#include <std_msgs/Float64.h>
#include <std_msgs/Int16.h>
#include <Encoder.h>

////////////////////////////////
// Pin Definitions
////////////////////////////////
#define Enc1_Pin1 2 // encoder1 CLK pin
#define Enc1_Pin2 3  // encoder1 DT pin

Encoder myEnc(Enc1_Pin1, Enc1_Pin2);

////////////////////////////////
// ROS Definitions
////////////////////////////////
ros::NodeHandle  nh;
//TODO: Make custom message type with all four encoder values
//TODO: Switch to type INT

std_msgs::Int16 encdr_1;
ros::Publisher pub_encdr_1("/Arduino/01/encoder1/val", &encdr_1);
// std_msgs::Int16 encdr_2;
// ros::Publisher pub_encdr_2("/Arduino/01/encoder2/val", &encdr_2);
// std_msgs::Int16 encdr_3;
// ros::Publisher pub_encdr_3("/Arduino/01/encoder3/val", &encdr_3);
// std_msgs::Int16 encdr_4;
// ros::Publisher pub_encdr_4("/Arduino/01/encoder4/val", &encdr_4);

////////////////////////////////
// Variable Definitions
////////////////////////////////
long oldPosition  = -999;

unsigned long int t_old=0, t=0.0, freq = 1000/60;

void setup()
{
  nh.initNode();
  nh.advertise(pub_encdr_1);
  // nh.advertise(pub_encdr_2);
  // nh.advertise(pub_encdr_3);
  // nh.advertise(pub_encdr_4);
}

void loop()
{
  t = millis();

  long newPosition = myEnc.read();
  if (newPosition != oldPosition) {
    oldPosition = newPosition;
    // Serial.println(newPosition);
    encdr_1.data = newPosition;
  }

  
  // encdr_2.data = counter_2;
  // encdr_3.data = counter_3;
  // encdr_4.data = counter_4;

  // Publish at a designated frequency in milliseconds
  if ((t-t_old) > freq)
  {
    pub_encdr_1.publish(&encdr_1);
    // pub_encdr_2.publish(&encdr_2);
    // pub_encdr_3.publish(&encdr_3);
    // pub_encdr_4.publish(&encdr_4);
    t_old = t;
  }
  nh.spinOnce();
}

 
 
