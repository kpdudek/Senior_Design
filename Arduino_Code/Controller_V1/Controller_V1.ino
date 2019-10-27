#include <ros.h>
#include <std_msgs/Float64.h>

////////////////////////////////
// Pin Definitions
////////////////////////////////
#define inputCLK_1 2 // encoder1 CLK pin
#define inputDT_1 3  // encoder1 DT pin
#define inputCLK_2 4 // encoder2 CLK pin
#define inputDT_2 5  // encoder2 DT pin
#define inputCLK_3 6 // encoder3 CLK pin
#define inputDT_3 7  // encoder3 DT pin
#define inputCLK_4 8 // encoder4 CLK pin
#define inputDT_4 9  // encoder4 DT pin

////////////////////////////////
// ROS Definitions
////////////////////////////////
ros::NodeHandle  nh;
//TODO: Make custom message type with all four encoder values
std_msgs::Float64 encdr_1;
ros::Publisher pub_encdr_1("/Arduino/01/encoder1/val", &encdr_1);
std_msgs::Float64 encdr_2;
ros::Publisher pub_encdr_2("/Arduino/01/encoder2/val", &encdr_2);
std_msgs::Float64 encdr_3;
ros::Publisher pub_encdr_3("/Arduino/01/encoder3/val", &encdr_3);
std_msgs::Float64 encdr_4;
ros::Publisher pub_encdr_4("/Arduino/01/encoder4/val", &encdr_4);

////////////////////////////////
// Variable Definitions
////////////////////////////////
int counter_1 = 0; 
int currentStateCLK_1;
int previousStateCLK_1; 
String encdir_1 = "";

int counter_2 = 0; 
int currentStateCLK_2;
int previousStateCLK_2; 
String encdir_2 = "";

int counter_3 = 0; 
int currentStateCLK_3;
int previousStateCLK_3; 
String encdir_3 = "";

int counter_4 = 0; 
int currentStateCLK_4;
int previousStateCLK_4; 
String encdir_4 = "";

unsigned long int t_old=0, t=0.0, freq = 1000/60;

void setup()
{
  nh.initNode();
  nh.advertise(pub_encdr_1);
  nh.advertise(pub_encdr_2);
  nh.advertise(pub_encdr_3);
  nh.advertise(pub_encdr_4);
  pinMode (inputCLK_1,INPUT);
  pinMode (inputDT_1,INPUT);
  pinMode (inputCLK_2,INPUT);
  pinMode (inputDT_2,INPUT);
  pinMode (inputCLK_3,INPUT);
  pinMode (inputDT_3,INPUT);
  pinMode (inputCLK_4,INPUT);
  pinMode (inputDT_4,INPUT);
  previousStateCLK_1 = digitalRead(inputCLK_1);
  previousStateCLK_2 = digitalRead(inputCLK_2);
  previousStateCLK_3 = digitalRead(inputCLK_3);
  previousStateCLK_4 = digitalRead(inputCLK_4);
}

void loop()
{
  t = millis();
  // Read the current state of inputCLK
  currentStateCLK_1 = digitalRead(inputCLK_1);
  currentStateCLK_2 = digitalRead(inputCLK_2);
  currentStateCLK_3 = digitalRead(inputCLK_3);
  currentStateCLK_4 = digitalRead(inputCLK_4);
  
  // Read encoder 1
  if (currentStateCLK_1 != previousStateCLK_1){ 
    // If the inputDT state is different than the inputCLK state then 
    // the encoder is rotating counterclockwise
    if (digitalRead(inputDT_1) != currentStateCLK_1)
    { 
      counter_1 --;
      encdir_1 ="CCW";
    } 
    else 
    {
      // Encoder is rotating clockwise
      counter_1 ++;
      encdir_1 ="CW";
    }
  }

  // Read encoder 2
  if (currentStateCLK_2 != previousStateCLK_2){ 
    // If the inputDT state is different than the inputCLK state then 
    // the encoder is rotating counterclockwise
    if (digitalRead(inputDT_2) != currentStateCLK_2)
    { 
      counter_2 --;
      encdir_2 ="CCW";
    } 
    else 
    {
      // Encoder is rotating clockwise
      counter_2 ++;
      encdir_2 ="CW";
    }
  }

  // Read encoder 3
  if (currentStateCLK_3 != previousStateCLK_3){ 
    // If the inputDT state is different than the inputCLK state then 
    // the encoder is rotating counterclockwise
    if (digitalRead(inputDT_3) != currentStateCLK_3)
    { 
      counter_3 --;
      encdir_3 ="CCW";
    } 
    else 
    {
      // Encoder is rotating clockwise
      counter_3 ++;
      encdir_3 ="CW";
    }
  }

  // Read encoder 4
  if (currentStateCLK_4 != previousStateCLK_4){ 
    // If the inputDT state is different than the inputCLK state then 
    // the encoder is rotating counterclockwise
    if (digitalRead(inputDT_4) != currentStateCLK_4)
    { 
      counter_4 --;
      encdir_4 ="CCW";
    } 
    else 
    {
      // Encoder is rotating clockwise
      counter_4 ++;
      encdir_4 ="CW";
    }
  }
  
  // Update previousStateCLK with the current state
  previousStateCLK_1 = currentStateCLK_1;
  previousStateCLK_2 = currentStateCLK_2;
  previousStateCLK_3 = currentStateCLK_3;
  previousStateCLK_4 = currentStateCLK_4;
  
  encdr_1.data = counter_1;
  encdr_2.data = counter_2;
  encdr_3.data = counter_3;
  encdr_4.data = counter_4;

  // Publish at a designated frequency in milliseconds
  if ((t-t_old) > freq)
  {
    pub_encdr_1.publish(&encdr_1);
    pub_encdr_2.publish(&encdr_2);
    pub_encdr_3.publish(&encdr_3);
    pub_encdr_4.publish(&encdr_4);
    t_old = t;
  }
  nh.spinOnce();
}

 
 
