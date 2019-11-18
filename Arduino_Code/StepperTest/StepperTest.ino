#define PUL 2
#define DIR 3
#define inputCLK_1 6 // encoder1 CLK pin
#define inputDT_1 5 // encoder1 DT pin


int counter_1 = 0, motorStep = 0,dir;
float setAngle = 0.0, motorAngle = 0.0,error;
int currentStateCLK_1;
int previousStateCLK_1; 
String encdir_1 = "";

unsigned long int t_old=0, t=0.0;
float freq = 700; // number of microseconds to wait between pulses

void setup() {
  // put your setup code here, to run once:
  pinMode(PUL,OUTPUT);
  pinMode(DIR,OUTPUT);

  pinMode(inputCLK_1,INPUT_PULLUP);
  pinMode(inputDT_1,INPUT_PULLUP);

  previousStateCLK_1 = digitalRead(inputCLK_1);
}

void loop() {
  
  read_encdr_1();
  
  setAngle = (counter_1 / 1200.0)*360;
  motorAngle = (motorStep / 800.0)*360;  
  
  error = setAngle - motorAngle;

  // TODO: Add FLP check to make error equality account for the discretization
  if (error > 0)
  {
    digitalWrite(DIR,LOW);
    dir = 1;
    stepp(dir);
  }
  else if(error < 0)
  {
    digitalWrite(DIR,HIGH);
    dir = -1;
    stepp(dir);
  }
}

void stepp(int dir)
{
  t = micros();
  if ((t-t_old) > freq)
  {
    digitalWrite(PUL,HIGH);
    delayMicroseconds(1);
    digitalWrite(PUL,LOW);
    delayMicroseconds(1);
    t_old = t;
    motorStep = motorStep + dir;
  }
}


void read_encdr_1()
{
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

  previousStateCLK_1 = currentStateCLK_1;
}
