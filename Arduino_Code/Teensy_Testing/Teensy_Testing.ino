#include <ros.h>
#include <std_msgs/Float64.h>
#include <AR3/AR3_Feedback.h>
#include <Teensy/AR3_Control.h>
#include <math.h>
#include <Encoder.h>
#include <Servo.h>

////////////////////////////////////////////////////////////////////////////////////////////////////////////
//  Pin definitions and Variables
//  All measuremements are in mm, all angles are in radians
////////////////////////////////////////////////////////////////////////////////////////////////////////////
#define joint1PUL 0
#define joint1DIR 1

#define joint2PUL 2
#define joint2DIR 3

#define joint3PUL 4
#define joint3DIR 5

#define joint4PUL 6
#define joint4DIR 7

#define joint5PUL 8
#define joint5DIR 9

#define joint6PUL 10
#define joint6DIR 11

#define joint1LIM 31

// E-Stop '+' pin. Must be grounded to work
#define eStopPin 32

// Good 'ol PI
float pi = 3.1415926;

// Varables used to store time both current and 'stopwatch' times
unsigned long int t_old=0, t=0, pub_freq = 1000000;

// Number of pulses to rotate a joint 2*PI radians. Accounts for settings on the stepper driver and
// all mechanical ratios. Inline comments denote the parameters used in the calculation.
float pulse1Rev = 1600.0*10.0*4.0; // pulse/rev, gearbox, pulley ratio
float pulse2Rev = 3200.0*50.0; // pulse/rev, gearbox
float pulse3Rev = 1600.0*50.0; // pulse/rev, gearbox
float pulse4Rev = 1600.0*(13.0+(212.0/289.0))*2.4893; // pulse/rev, gearbox, pulley ratio
float pulse5Rev = 1.0/((8.0/1600.0)/(2.0*pi*13.675)); // pulse/rev, lead screw
float pulse6Rev = 800.0*(19.0+(38.0/187.0)); // pulse/rev, gearbox;

// Pulse width of the signal sent to the stepper driver. This time is in microsecons
// and is passed to the delay_microseconds() function. Making this value larger will
// severely affect the frequency at which the main loop runs
int pulDelay = 50; // default is 50 for max speed

// Joint angle variables
float SetAngles[6] = {0.0,0.0,0.0,0.0,0.0,0.0};
float AngleErrors[6] = {0.0,0.0,0.0,0.0,0.0,0.0};
float JointAngles[6] = {0.0,0.0,0.0,0.0,0.0,0.0};

// Encoder pins and classes
#define Enc1_Pin1 22 // encoder1 CLK pin
#define Enc1_Pin2 23  // encoder1 DT pin
Encoder Enc1(Enc1_Pin1, Enc1_Pin2);
long oldPosition1  = -1, newPosition1 = 0;

// Fuzzy equality to prevent jitter in the motors. This value is when the joint is 'close enough'
// to its setpoint
float angleTol = 0.0005;

// Acceleration time. Time to add between pulses to "decelerate" the joint
float dist[6] = {0.0,0.0,0.0,0.0,0.0,0.0};
float accelTime[6] = {0.0,0.0,0.0,0.0,0.0,0.0};
float t_old_accel[6] = {0.0,0.0,0.0,0.0,0.0,0.0};
float ka = 800.0, thresh = 1.57;

// Current state of the e-stop pin. Default is True meaning no movements will not occur
int ePinValue = 1;
int limState = 0;

// Command States
int home = 0;
int run = 0;
int rest = 0;

// Gripper positions
Servo gripper_servo;
int gripper_cmd = 0, closed_pos = 1, opened_pos = 80;

////////////////////////////////////////////////////////////////////////////////////////////////////////////
// ROS Definitions
////////////////////////////////////////////////////////////////////////////////////////////////////////////
// Joint 1 callback
ros::NodeHandle  nh;

void AR3ControlCallback(const Teensy::AR3_Control &AR3_Control_Data){
    memcpy(SetAngles,AR3_Control_Data.joint_angles,sizeof(SetAngles));
    home = AR3_Control_Data.home;
    run = AR3_Control_Data.run;
    rest = AR3_Control_Data.rest;
    gripper_cmd = AR3_Control_Data.close_gripper;
}

ros::Subscriber<Teensy::AR3_Control> AR3ControlSub("/AR3/Control",& AR3ControlCallback);

AR3::AR3_Feedback AR3FeedbackData;
ros::Publisher AR3FeedbackPub("/AR3/Feedback", &AR3FeedbackData);

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

    pinMode(joint1LIM,INPUT_PULLUP);

    gripper_servo.attach(21,500,2400);

    // Ros subscribers
    nh.initNode();
    nh.subscribe(AR3ControlSub);

    // Debug info
    nh.advertise(AR3FeedbackPub);
}

////////////////////////////////////////////////////////////////////////////////////////////////////////////
//  Main Loop
////////////////////////////////////////////////////////////////////////////////////////////////////////////
void loop() {
    t = micros();
    ePinValue = digitalRead(eStopPin);
    newPosition1 = Enc1.read();
    if (newPosition1 != oldPosition1) {
      AR3FeedbackData.encoder_pulses[1] = int(newPosition1);
      oldPosition1 = newPosition1;
      JointAngles[1] = (float(newPosition1) / (512.0*50.0*4.0)) * 2.0*pi;
      // Ensure angle in range 0 --> 360
      if(JointAngles[1] >= (2.0*pi)){JointAngles[1] = JointAngles[1] - (2.0*pi);}
      if(JointAngles[1] < 0.0){JointAngles[1] = (2.0*pi) + JointAngles[1];}
    }

    if (gripper_cmd == 1){
      gripper_servo.write(opened_pos);
      AR3FeedbackData.gripper_closed = 1;
    }
    else{
      gripper_servo.write(closed_pos);
      AR3FeedbackData.gripper_closed = 0;
    }
    
    // E-Stop state
    if (ePinValue){
        if ((t-t_old) > (pub_freq)){
            AR3FeedbackData.eStop = 1;
            AR3FeedbackData.running = 0;
            arry_cpy(AR3FeedbackData.joint_angles,JointAngles,6);
            arry_cpy(AR3FeedbackData.setpoint_angles,SetAngles,6);
            AR3FeedbackPub.publish(&AR3FeedbackData);
            t_old = t;
        }
    }

    // Run state
    else if (!ePinValue && run){
        AR3FeedbackData.eStop = 0;
        AR3FeedbackData.running = 1;

        //TODO: Function that calculates all accel times
        
        // Joint 1
        dist[0] = abs(edgeAngle(SetAngles[0],JointAngles[0]));
        if (dist[0] > 3.14){dist[0] = dist[0] - 3.14;}
        if(dist[0]<thresh){accelTime[0] = pow((1.0/dist[0]),2.0);}
        else{accelTime[0] = 0.0;}

        if ((t-t_old_accel[0]) > accelTime[0]){
            AngleErrors[0] = SetAngles[0] - JointAngles[0];
            moveJoint(joint1PUL,joint1DIR,&JointAngles[0],AngleErrors[0],pulse1Rev);
            t_old_accel[0] = t;
        }

        // Joint 2
        dist[1] = abs(edgeAngle(SetAngles[1],JointAngles[1]));
        if (dist[1] > 3.14){dist[1] = dist[1] - 3.14;}
        if(dist[1]<thresh){accelTime[1] = pow((1.0/dist[1]),2.0);}
        else{accelTime[1] = 0.0;}
    
        if ((t-t_old_accel[1]) > accelTime[1]){
            AngleErrors[1] = SetAngles[1] - JointAngles[1];
            cl_moveJoint(joint2PUL,joint2DIR,AngleErrors[1]);
            t_old_accel[1] = t;
        }
        
        //  Joint 3
        dist[2] = abs(edgeAngle(SetAngles[2],JointAngles[2]));
        if (dist[2] > 3.14){dist[2] = dist[2] - 3.14;}
        if(dist[2]<thresh){accelTime[2] = pow((1.0/dist[2]),2.0);}
        else{accelTime[2] = 0.0;}

        if ((t-t_old_accel[2]) > accelTime[2]){
            AngleErrors[2] = SetAngles[2] - JointAngles[2];
            moveJoint(joint3PUL,joint3DIR,&JointAngles[2],AngleErrors[2],pulse3Rev);
            t_old_accel[2] = t;
        }

        //  Joint 4
        AngleErrors[3] = SetAngles[3] - JointAngles[3];
        moveJoint(joint4PUL,joint4DIR,&JointAngles[3],AngleErrors[3],pulse4Rev);

        //  Joint 5
        dist[4] = abs(edgeAngle(SetAngles[4],JointAngles[4]));
        if (dist[4] > 3.14){dist[4] = dist[4] - 3.14;}
        if(dist[4]<thresh){accelTime[4] = pow((1.0/dist[4]),2.0);}
        else{accelTime[4] = 0.0;}

        if ((t-t_old_accel[4]) > accelTime[4]){
            AngleErrors[4] = SetAngles[4] - JointAngles[4];
            moveJoint(joint5PUL,joint5DIR,&JointAngles[4],AngleErrors[4],pulse5Rev);
            t_old_accel[4] = t;
        }

        //  Joint 6
        AngleErrors[5] = SetAngles[5] - JointAngles[5];
        moveJoint(joint6PUL,joint6DIR,&JointAngles[5],AngleErrors[5],pulse6Rev);

        // Publish the arduinos current angle value for debugging purposes
        // COMMENT OUT FOR SPEED IMPROVEMENT
        if ((t-t_old) > (pub_freq)){
            arry_cpy(AR3FeedbackData.joint_angles,JointAngles,6);
            arry_cpy(AR3FeedbackData.setpoint_angles,SetAngles,6);
            AR3FeedbackPub.publish(&AR3FeedbackData);
            t_old = t;
        }
    }

    // Homing state
    else if(!ePinValue && home){
        AR3FeedbackData.running = 0;
        AR3FeedbackData.eStop = 0;
        limState = digitalRead(joint1LIM);
        
        if (!limState && !AR3FeedbackData.homed){
            homeJoint(joint1PUL,joint1DIR,1);
            AR3FeedbackData.homed = 0;
        }
        else{
            JointAngles[0] = (2.0*pi)-(pi/2.0);
            AR3FeedbackData.homed = 1;
        }

        if ((t-t_old) > (pub_freq)){
            arry_cpy(AR3FeedbackData.joint_angles,JointAngles,6);
            arry_cpy(AR3FeedbackData.setpoint_angles,SetAngles,6);
            AR3FeedbackPub.publish(&AR3FeedbackData);
            t_old = t;
        }

    }

    // No state assigned and no E-Stop
    else{
        if ((t-t_old) > (pub_freq)){
            AR3FeedbackData.eStop = 0;
            AR3FeedbackData.running = 0;
            arry_cpy(AR3FeedbackData.joint_angles,JointAngles,6);
            arry_cpy(AR3FeedbackData.setpoint_angles,SetAngles,6);
            AR3FeedbackPub.publish(&AR3FeedbackData);
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

void moveJoint(int pin, int dirPin, float* angle, float error, float pulRev) {
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

void cl_moveJoint(int pin, int dirPin, float error) {
    // Setpoint leads current
    if(error > angleTol && error <= pi){
        sendPulse(pin,dirPin,0);
    }
    else if(error > angleTol){
        sendPulse(pin,dirPin,1);
    }
    // Setpoint trails current
    else if(error < -angleTol && error >= -pi){
        sendPulse(pin,dirPin,1);
    }
    else if(error < -angleTol){
        sendPulse(pin,dirPin,0);
    }
}

void homeJoint(int pin, int dirPin, int dir){
    sendPulse(pin,dirPin,dir);
    delayMicroseconds(400);
}

void arry_cpy(float ros[], float local[], int len) {
  for(int i=0; i<len; i++) {
    ros[i] = local[i];
  }
}

float edgeAngle(float angle1, float angle2){
    // function [edgeAngle]=edge_angle(vertex0,vertex1,vertex2,angleType)
    // vec1=(vertex1-vertex0)/norm(vertex1-vertex0);%compute the normalized vector from vertex0 to vertex1.
    // vec2=(vertex2-vertex0)/norm(vertex2-vertex0);%compute the normalized vector from vertex0 to vertex2.

    // cosine_theta=vec1'*vec2;%compute the cosine value of the angle using the inner product of two vectors.
    // sine_theta=[0 0 1]*cross([vec1;0],[vec2;0]);%compute the sine value of the angle.

    // edgeAngle=atan2(sine_theta,cosine_theta);

    // %compute the edge angle
    // if strcmpi(angleType,'unsigned')
    //     edgeAngle=mod(edgeAngle+2*pi,2*pi);
    // elseif strcmpi(angleType,'signed')
    //     return;
    // else
    //     disp('input error');
    //     edgeAngle=NaN;
    //     return;
    // end
    float vertex0[2] = {0.0,0.0};
    float vertex1[2];
    float vertex2[2];
    vertex1[0] = cos(angle1);
    vertex1[1] = sin(angle1);
    vertex2[0] = cos(angle2);
    vertex2[1] = sin(angle2);

    float cosine_theta = vertex1[0]*vertex2[0] + vertex1[1]*vertex2[1];
    float sine_theta = vertex1[0]*vertex2[1]- vertex1[1]*vertex2[0];
    float edge_angle = atan2(sine_theta,cosine_theta);
    return edge_angle;

}
