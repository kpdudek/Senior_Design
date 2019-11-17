#define PUL 11
#define DIR 12
#define inputCLK_1 10 // encoder1 CLK pin
#define inputDT_1 13 // encoder1 DT pin


int counter_1 = 0, motorStep = 0;
float setAngle = 0.0, motorAngle = 0.0,error;
int currentStateCLK_1;
int previousStateCLK_1; 
String encdir_1 = "";

void setup() {
  // put your setup code here, to run once:
  pinMode(PUL,OUTPUT);
  pinMode(DIR,OUTPUT);

  pinMode(inputCLK_1,INPUT_PULLUP);
  pinMode(inputDT_1,INPUT_PULLUP);

  previousStateCLK_1 = digitalRead(inputCLK_1);
  //Serial.begin(9600);
}

void loop() {
  
  read_encdr_1();
  
  setAngle = (counter_1 / 1200.0);
  motorAngle = (motorStep / 800.0);  
  
//  Serial.print(setAngle);Serial.print("\t");
//  Serial.print(motorAngle);Serial.print("\t");
  
  error = setAngle - motorAngle;
  
  if (error > 0)
  {
    digitalWrite(DIR,LOW);
    stepp();
    motorStep++;
  }
  else if(error < 0)
  {
    digitalWrite(DIR,HIGH);
    stepp();
    motorStep--;
  }
  
//  Serial.println();
}

void stepp()
{
  digitalWrite(PUL,HIGH);
  delayMicroseconds(100);
  digitalWrite(PUL,LOW);
  delayMicroseconds(100);
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
