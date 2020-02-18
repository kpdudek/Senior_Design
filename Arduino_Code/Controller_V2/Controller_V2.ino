
#include <ros.h>
//#include <std_msgs/Float64.h>
//#include <std_msgs/Int16.h>
#include <Teensy/Teensy_Pulses.h>
#include <Encoder.h>

////////////////////////////////
// Pin Definitions
////////////////////////////////
#define Enc1_Pin1 2 // encoder1 CLK pin
#define Enc1_Pin2 3  // encoder1 DT pin

Encoder Enc1(Enc1_Pin1, Enc1_Pin2);

#define Enc2_Pin1 4 // encoder2 CLK pin
#define Enc2_Pin2 5  // encoder2 DT pin

Encoder Enc2(Enc2_Pin1, Enc2_Pin2);

#define Enc3_Pin1 6 // encoder3 CLK pin
#define Enc3_Pin2 7  // encoder3 DT pin

Encoder Enc3(Enc3_Pin1, Enc3_Pin2);

#define Enc4_Pin1 8 // encoder4 CLK pin
#define Enc4_Pin2 9  // encoder4 DT pin

Encoder Enc4(Enc4_Pin1, Enc4_Pin2);

////////////////////////////////
// ROS Definitions
////////////////////////////////
ros::NodeHandle  nh;

Teensy::Teensy_Pulses pulse_msg;
ros::Publisher pub_pulses("/Teensy/encoders", &pulse_msg);

////////////////////////////////
// Variable Definitions
////////////////////////////////
long oldPosition1  = -999;
long oldPosition2  = -999;
long oldPosition3  = -999;
long oldPosition4  = -999;

unsigned long int t_old=0, t=0.0, freq = 1000/60;

void setup()
{
  nh.initNode();
  nh.advertise(pub_pulses);
}

void loop()
{
  t = millis();

  long newPosition1 = Enc1.read();
  if (newPosition1 != oldPosition1) {
    oldPosition1 = newPosition1;
    pulse_msg.j1_pulses = newPosition1;
  }
  long newPosition2 = Enc2.read();
  if (newPosition2 != oldPosition2) {
    oldPosition2 = newPosition2;
    pulse_msg.j2_pulses = newPosition2;
  }
  
  long newPosition3 = Enc3.read();
  if (newPosition3 != oldPosition3) {
    oldPosition3 = newPosition3;
    pulse_msg.j3_pulses = newPosition3;
  }
  
  long newPosition4 = Enc4.read();
  if (newPosition4 != oldPosition4) {
    oldPosition4 = newPosition4;
    pulse_msg.j4_pulses = newPosition4;
  }
  
  pulse_msg.j5_pulses = 5;
  pulse_msg.j6_pulses = 6;
     

  // Publish at a designated frequency in milliseconds
  if ((t-t_old) > freq)
  {
    pub_pulses.publish(&pulse_msg);
    // pub_encdr_1.publish(&encdr_1);
    // pub_encdr_2.publish(&encdr_2);
    // pub_encdr_3.publish(&encdr_3);
    // pub_encdr_4.publish(&encdr_4);
    t_old = t;
  }
  nh.spinOnce();
}

 
 
