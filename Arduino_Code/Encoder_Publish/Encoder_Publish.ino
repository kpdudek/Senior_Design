#include <ros.h>
#include <std_msgs/Float64.h>

////////////////////////////////
// Pin Definitions
////////////////////////////////
#define inputCLK_1 5 // encoder CLK pin
#define inputDT_1 4  // encoder DT pin
#define inputCLK_2 13 // encoder CLK pin
#define inputDT_2 12  // encoder DT pin

////////////////////////////////
// ROS Definitions
////////////////////////////////
ros::NodeHandle  nh;
std_msgs::Float64 encdr_1;
ros::Publisher pub_encdr_1("/Arduino/01/encoder1/val", &encdr_1);
std_msgs::Float64 encdr_2;
ros::Publisher pub_encdr_2("/Arduino/01/encoder2/val", &encdr_2);

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

unsigned long int t_old=0, t=0.0, freq = 1000/60;

void setup()
{
  nh.initNode();
  nh.advertise(pub_encdr_1);
  nh.advertise(pub_encdr_2);
  pinMode (inputCLK_1,INPUT);
  pinMode (inputDT_1,INPUT);
  pinMode (inputCLK_1,INPUT);
  pinMode (inputDT_1,INPUT);
  previousStateCLK_1 = digitalRead(inputCLK_1);
  previousStateCLK_2 = digitalRead(inputCLK_2);
}

void loop()
{
  t = millis();
  // Read the current state of inputCLK
  currentStateCLK_1 = digitalRead(inputCLK_1);
  currentStateCLK_2 = digitalRead(inputCLK_2);
  
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
   
  // Update previousStateCLK with the current state
  previousStateCLK_1 = currentStateCLK_1;
  previousStateCLK_2 = currentStateCLK_2;
  
  encdr_1.data = counter_1;
  encdr_2.data = counter_2;
  if ((t-t_old) > freq)
  {
    pub_encdr_1.publish(&encdr_1);
    pub_encdr_2.publish(&encdr_2);
    t_old = t;
  }
  nh.spinOnce();
}

 
 
