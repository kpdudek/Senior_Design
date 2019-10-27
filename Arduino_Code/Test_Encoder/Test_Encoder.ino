
////////////////////////////////
// Pin Definitions
////////////////////////////////
#define inputCLK_1 8 // encoder1 CLK pin
#define inputDT_1 9  // encoder1 DT pin

////////////////////////////////
// Variable Definitions
////////////////////////////////
int counter_1 = 0; 
int currentStateCLK_1;
int previousStateCLK_1; 
String encdir_1 = "";

unsigned long int t_old=0, t=0.0, freq = 1000/60;

void setup()
{
  pinMode (inputCLK_1,INPUT);
  pinMode (inputDT_1,INPUT);
  previousStateCLK_1 = digitalRead(inputCLK_1);
  Serial.begin(9600);
}

void loop()
{
  t = millis();
  // Read the current state of inputCLK
  currentStateCLK_1 = digitalRead(inputCLK_1);
  
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

  
  // Update previousStateCLK with the current state
  previousStateCLK_1 = currentStateCLK_1;

  // Publish at a designated frequency in milliseconds
  if ((t-t_old) > freq)
  {
    Serial.println(counter_1);
    t_old = t;
  }
}

 
 
