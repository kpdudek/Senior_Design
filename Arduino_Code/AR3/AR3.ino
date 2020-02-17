#include <ros.h>
#include <std_msgs/Float64.h>
#include <AR3/AR3_Debug.h>
#include <math.h>

////////////////////////////////////////////////////////////////////////////////////////////////////////////
//  Pin definitions and Variables
//  All measuremements are in mm, all angles are in radians
////////////////////////////////////////////////////////////////////////////////////////////////////////////
#define joint1PUL 22
#define joint1DIR 23

#define joint2PUL 24
#define joint2DIR 25

#define joint3PUL 26
#define joint3DIR 27

#define joint4PUL 28
#define joint4DIR 29

#define joint5PUL 30
#define joint5DIR 31

#define joint6PUL 32
#define joint6DIR 33

// E-Stop '+' pin. Must be grounded to work
#define eStopPin 13

// Good 'ol PI
double pi = 3.1415926;

// Varables used to store time both current and 'stopwatch' times
unsigned long int t_old=0, t=0.0, t_old_accel = 0;

// Number of pulses to rotate a joint 2*PI radians. Accounts for settings on the stepper driver and
// all mechanical ratios. Inline comments denote the parameters used in the calculation.
double pulse1Rev = 1600.0*10.0*4.0; // pulse/rev, gearbox, pulley ratio
double pulse2Rev = 3200.0*50.0; // pulse/rev, gearbox
double pulse3Rev = 1600.0*50.0; // pulse/rev, gearbox
double pulse4Rev = 1600.0*(13.0+(212.0/289.0))*2.4893; // pulse/rev, gearbox, pulley ratio
double pulse5Rev = 1/((8.0/1600.0)/(2.0*pi*13.675)); // pulse/rev, lead screw
double pulse6Rev = 1600.0*(19.0+(38.0/187.0)); // pulse/rev, gearbox;

// Pulse width of the signal sent to the stepper driver. This time is in microsecons
// and is passed to the delay_microseconds() function. Making this value larger will
// severely affect the frequency at which the main loop runs
int pulDelay = 80; // default is 50 for max speed

// Joint angle variables
double joint1Angle = 0.0, joint1SetAngle = 0.0, joint1Error = 0.0;
double joint2Angle = 0.0, joint2SetAngle = 0.0, joint2Error = 0.0;
double joint3Angle = 0.0, joint3SetAngle = 0.0, joint3Error = 0.0;
double joint4Angle = 0.0, joint4SetAngle = 0.0, joint4Error = 0.0;
double joint5Angle = 0.0, joint5SetAngle = 0.0, joint5Error = 0.0;
double joint6Angle = 0.0, joint6SetAngle = 0.0, joint6Error = 0.0;

// Fuzzy equality to prevent jitter in the motors. This value is when the joint is 'close enough'
// to its setpoint
double angleTol = 0.0005;

// Acceleration time. Time to add between pulses to "decelerate" the joint
double accelTime = 0;

// Current state of the e-stop pin. Default is True meaning no movements will occur
int ePinValue = 1;

////////////////////////////////////////////////////////////////////////////////////////////////////////////
// ROS Definitions
////////////////////////////////////////////////////////////////////////////////////////////////////////////
AR3::AR3_Debug jointPubData;
ros::Publisher jointPub("/AR3/Debug", &jointPubData);

// Joint 1 callback
void joint1Callback(const std_msgs::Float64& joint1_msg){
    joint1SetAngle = joint1_msg.data;// * 57.2957795;
}

// Joint 2 callback
void joint2Callback(const std_msgs::Float64& joint2_msg){
    joint2SetAngle = joint2_msg.data;// * 57.2957795;
}

// Joint 3 callback
void joint3Callback(const std_msgs::Float64& joint3_msg){
    joint3SetAngle = joint3_msg.data;// * 57.2957795;
}

// Joint 4 callback
void joint4Callback(const std_msgs::Float64& joint4_msg){
    joint4SetAngle = joint4_msg.data;// * 57.2957795;
}

// Joint 5 callback
void joint5Callback(const std_msgs::Float64& joint5_msg){
    joint5SetAngle = joint5_msg.data;// * 57.2957795;
}

// Joint 6 callback
void joint6Callback(const std_msgs::Float64& joint6_msg){
    joint6SetAngle = joint6_msg.data;// * 57.2957795;
}

ros::NodeHandle  nh;
ros::Subscriber<std_msgs::Float64> joint1Sub("/rrbot/joint1_position_controller/command",& joint1Callback);
ros::Subscriber<std_msgs::Float64> joint2Sub("/rrbot/joint2_position_controller/command",& joint2Callback);
ros::Subscriber<std_msgs::Float64> joint3Sub("/rrbot/joint3_position_controller/command",& joint3Callback);
ros::Subscriber<std_msgs::Float64> joint4Sub("/rrbot/joint4_position_controller/command",& joint4Callback);
ros::Subscriber<std_msgs::Float64> joint5Sub("/rrbot/joint5_position_controller/command",& joint5Callback);
ros::Subscriber<std_msgs::Float64> joint6Sub("/rrbot/joint6_position_controller/command",& joint6Callback);

////////////////////////////////////////////////////////////////////////////////////////////////////////////
//  Setup
////////////////////////////////////////////////////////////////////////////////////////////////////////////
void setup() {
    // Stepper Motors
    pinMode(joint1PUL,OUTPUT);
    pinMode(joint1DIR,OUTPUT);

    pinMode(joint2PUL,OUTPUT);
    pinMode(joint2DIR,OUTPUT);

    pinMode(joint3PUL,OUTPUT);
    pinMode(joint3DIR,OUTPUT);

    pinMode(joint4PUL,OUTPUT);
    pinMode(joint4DIR,OUTPUT);

    pinMode(joint5PUL,OUTPUT);
    pinMode(joint5DIR,OUTPUT);

    pinMode(joint6PUL,OUTPUT);
    pinMode(joint6DIR,OUTPUT);

    // E-Stop pin
    pinMode(eStopPin,INPUT_PULLUP);

    // Ros subscribers
    nh.initNode();
    nh.subscribe(joint1Sub);
    nh.subscribe(joint2Sub);
    nh.subscribe(joint3Sub);
    nh.subscribe(joint4Sub);
    nh.subscribe(joint5Sub);
    nh.subscribe(joint6Sub);

    // Debug info
    nh.advertise(jointPub);
}

////////////////////////////////////////////////////////////////////////////////////////////////////////////
//  Main Loop
////////////////////////////////////////////////////////////////////////////////////////////////////////////
void loop() {
    ePinValue = digitalRead(eStopPin);
    t = micros();

    if (!ePinValue){
        jointPubData.eStop = 0;
        
        // Joint 2
        joint1Error = joint1SetAngle - joint1Angle;
        moveJoint(joint1PUL,joint1DIR,&joint1Angle,joint1Error,pulse1Rev);

        // Joint 2
        joint2Error = joint2SetAngle - joint2Angle;
        moveJoint(joint2PUL,joint2DIR,&joint2Angle,joint2Error,pulse2Rev);

        //  Joint 3
        joint3Error = joint3SetAngle - joint3Angle;
        moveJoint(joint3PUL,joint3DIR,&joint3Angle,joint3Error,pulse3Rev);

        //  Joint 4
        joint4Error = joint4SetAngle - joint4Angle;
        moveJoint(joint4PUL,joint4DIR,&joint4Angle,joint4Error,pulse4Rev);

        //  Joint 5
        joint5Error = joint5SetAngle - joint5Angle;
        moveJoint(joint5PUL,joint5DIR,&joint5Angle,joint5Error,pulse5Rev);

        //  Joint 6
        joint6Error = joint6SetAngle - joint6Angle;
        moveJoint(joint6PUL,joint6DIR,&joint6Angle,joint6Error,pulse6Rev);

        // Publish the arduinos current angle value for debugging purposes
        // COMMENT OUT FOR SPEED IMPROVEMENT
        if ((t-t_old) > (2000000)){
            jointPubData.j1_current_angle = joint1Angle;
            jointPubData.j1_setpoint_angle = joint1SetAngle;

            jointPubData.j2_current_angle = joint2Angle;
            jointPubData.j2_setpoint_angle = joint2SetAngle;

            jointPubData.j3_current_angle = joint3Angle;
            jointPubData.j3_setpoint_angle = joint3SetAngle;

            jointPubData.j4_current_angle = joint4Angle;
            jointPubData.j4_setpoint_angle = joint4SetAngle;

            jointPubData.j5_current_angle = joint5Angle;
            jointPubData.j5_setpoint_angle = joint5SetAngle;

            jointPubData.j6_current_angle = joint6Angle;
            jointPubData.j6_setpoint_angle = joint6SetAngle;
            
            jointPub.publish(&jointPubData);
            t_old = t;
        }
    }
    else{
        if ((t-t_old) > (2000000)){
            jointPubData.eStop = 1;

            jointPubData.j1_current_angle = joint1Angle;
            jointPubData.j1_setpoint_angle = joint1SetAngle;
            
            jointPubData.j2_current_angle = joint2Angle;
            jointPubData.j2_setpoint_angle = joint2SetAngle;

            jointPubData.j3_current_angle = joint3Angle;
            jointPubData.j3_setpoint_angle = joint3SetAngle;

            jointPubData.j4_current_angle = joint4Angle;
            jointPubData.j4_setpoint_angle = joint4SetAngle;

            jointPubData.j5_current_angle = joint5Angle;
            jointPubData.j5_setpoint_angle = joint5SetAngle;

            jointPubData.j6_current_angle = joint6Angle;
            jointPubData.j6_setpoint_angle = joint6SetAngle;
            
            jointPub.publish(&jointPubData);
            t_old = t;
        }
    }
    
    // Update subscribers
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
    digitalWrite(dirPin,LOW);
  }
  else{
    digitalWrite(dirPin,HIGH);
  }
  
  digitalWrite(pin,HIGH);
  delayMicroseconds(pulDelay);
  digitalWrite(pin,LOW);
}

void moveJoint(int pin, int dirPin, double* angle, double error, double pulRev) {
    // Setpoint leads current
    if(error > angleTol && error <= pi){
        sendPulse(pin,dirPin,0);
        *angle = *angle + ((1.0/pulRev)*(2.0*pi));
    }
    else if(error > angleTol){
        sendPulse(pin,dirPin,1);
        *angle = *angle - ((1.0/pulRev)*(2.0*pi));
    }
    // Setpoint trails current
    else if(error < -angleTol && error >= -pi){
        sendPulse(pin,dirPin,1);
        *angle = *angle - ((1.0/pulRev)*(2.0*pi));
    }
    else if(error < -angleTol){
        sendPulse(pin,dirPin,0);
        *angle = *angle + ((1.0/pulRev)*(2.0*pi));
    }
    // Ensure angle in range 0 --> 360
    if(*angle >= (2.0*pi)){*angle = *angle - (2.0*pi);}
    if(*angle < 0.0){*angle = (2.0*pi) + *angle;}
}
