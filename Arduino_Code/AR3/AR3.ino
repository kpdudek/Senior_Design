#include <ros.h>
#include <std_msgs/Float64.h>
#include <math.h>
#include <AR3/AR3_Debug.h>

///////////////////////////////////////////////
//  Pin definitions and Variables
///////////////////////////////////////////////
#define PI 3.14159

#define joint1PUL 2
#define joint1DIR 3

unsigned long int t_old=0, t=0.0, freq = 100;
int pulDelay = 300;
float pulseRev = 800, angleTol = .01;

// Joint angle variables
float joint1Angle = 0, joint1SetAngle = 0, joint1Error = 0;

////////////////////////////////
// ROS Definitions
////////////////////////////////
AR3::AR3_Debug joint1PubData;
ros::Publisher joint1Pub("/AR3/joint1/debug", &joint1PubData);

void joint1Callback(const std_msgs::Float64& joint1_msg){
    joint1SetAngle = joint1_msg.data;// * 57.2957795;

    // joint1PubData.data = joint1Angle;
    // joint1PubData.data = joint1Error;
    // joint1Pub.publish(&joint1PubData);
}

ros::NodeHandle  nh;
ros::Subscriber<std_msgs::Float64> joint1Sub("/rrbot/joint1_position_controller/command",& joint1Callback);

///////////////////////////////////////////////
//  Setup
///////////////////////////////////////////////
void setup() {
    // put your setup code here, to run once:
    pinMode(joint1PUL,OUTPUT);
    pinMode(joint1DIR,OUTPUT);

    nh.initNode();
    nh.subscribe(joint1Sub);
    nh.advertise(joint1Pub);
}

///////////////////////////////////////////////
//  Main Loop
///////////////////////////////////////////////
void loop() {
    // TODO: calibration curve that goes from pulse time to rpm
    t = millis();

    joint1Error = joint1SetAngle - joint1Angle;
    // joint1Error = edgeAngle(joint1SetAngle,joint1Angle);
    // Setpoint leads current
    if(joint1Error > angleTol && joint1Error < PI){
        sendPulse(joint1PUL,joint1DIR,0);
        joint1Angle = joint1Angle + ((1.0/pulseRev)*(2*PI));
    }
    else if(joint1Error > angleTol){
        sendPulse(joint1PUL,joint1DIR,1);
        joint1Angle = joint1Angle - ((1.0/pulseRev)*(2*PI));
    }
    // Setpoint trails current
    else if(joint1Error < -angleTol && joint1Error > -PI){
        sendPulse(joint1PUL,joint1DIR,1);
        joint1Angle = joint1Angle - ((1.0/pulseRev)*(2*PI));
    }
    else if(joint1Error < -angleTol){
        sendPulse(joint1PUL,joint1DIR,0);
        joint1Angle = joint1Angle + ((1.0/pulseRev)*(2*PI));
    }

    // Ensure angle in range 0 --> 360
    if(joint1Angle >= (2*PI)){joint1Angle = joint1Angle - (2*PI);}
    if(joint1Angle < 0){joint1Angle = (2*PI) + joint1Angle;}

    // Publish the arduinos current angle value for debugging purposes
    // COMMENT OUT FOR SPEED IMPROVEMENT
    if ((t-t_old) > (1500)){
        joint1PubData.j1_current_angle = joint1Angle;
        joint1PubData.j1_setpoint_angle = joint1SetAngle;
        joint1PubData.j1_angle_error = joint1Error;
        joint1Pub.publish(&joint1PubData);
        t_old = t;
    }

    nh.spinOnce();
}

////////////////////////////////////////////////////////////
//  Send pulse to stepper motor
//  Pass the pulse pin, direction pin, and desired direction
//    --> 1: cw
//    --> 0: ccw
////////////////////////////////////////////////////////////
void sendPulse(int pin, int dirPin, int dir) {
  if (dir == 1){
    digitalWrite(dirPin,HIGH);
  }
  else{
    digitalWrite(dirPin,LOW);
  }
  
  digitalWrite(pin,HIGH);
  delayMicroseconds(pulDelay);
  digitalWrite(pin,LOW);
}
