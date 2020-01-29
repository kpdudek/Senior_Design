#include <ros.h>
#include <std_msgs/Float64.h>
#include <math.h>

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
float joint1Angle = 0, joint1SetAngle = 0, joint1Error;

////////////////////////////////
// ROS Definitions
////////////////////////////////
std_msgs::Float64 joint1PubData;
ros::Publisher joint1Pub("/AR3/joint1/current_angle", &joint1PubData);

void joint1Callback(const std_msgs::Float64& joint1_msg){
    joint1SetAngle = joint1_msg.data;// * 57.2957795;

    joint1PubData.data = joint1Angle;
    // joint1PubData.data = joint1Error;
    joint1Pub.publish(&joint1PubData);
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
    // if ((t-t_old) > (1000/25)){
    //     joint1PubData.data = joint1Angle;
    //     // joint1PubData.data = joint1Error;
    //     joint1Pub.publish(&joint1PubData);
    // }

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

// float edgeAngle(float angle1, float angle2){
//     // function [edgeAngle]=edge_angle(vertex0,vertex1,vertex2,angleType)
//     // vec1=(vertex1-vertex0)/norm(vertex1-vertex0);%compute the normalized vector from vertex0 to vertex1.
//     // vec2=(vertex2-vertex0)/norm(vertex2-vertex0);%compute the normalized vector from vertex0 to vertex2.

//     // cosine_theta=vec1'*vec2;%compute the cosine value of the angle using the inner product of two vectors.
//     // sine_theta=[0 0 1]*cross([vec1;0],[vec2;0]);%compute the sine value of the angle.

//     // edgeAngle=atan2(sine_theta,cosine_theta);

//     // %compute the edge angle
//     // if strcmpi(angleType,'unsigned')
//     //     edgeAngle=mod(edgeAngle+2*pi,2*pi);
//     // elseif strcmpi(angleType,'signed')
//     //     return;
//     // else
//     //     disp('input error');
//     //     edgeAngle=NaN;
//     //     return;
//     // end
//     float vertex0[2] = {0,0};
//     float vertex1[2];
//     float vertex2[2];
//     vertex1[0] = cos(angle1);
//     vertex1[1] = sin(angle1);
//     vertex2[0] = cos(angle2);
//     vertex2[1] = sin(angle2);

//     float cosine_theta = vertex1[0]*vertex2[0] + vertex1[1]*vertex2[1]; 
//     float sine_theta = vertex1[0]*vertex2[1]- vertex1[1]*vertex2[0];
//     float edge_angle = atan2(sine_theta,cosine_theta);
//     return edge_angle;

// }