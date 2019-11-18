#define PUL 2
#define DIR 3

int counter_1 = 0, motorStep = 0, dir;
float setAngle = 0.0, motorAngle = 0.0,error;

unsigned long int t_old=0, t=0.0;
float freq = 800; // number of microseconds to wait between pulses

void setup() {
  // put your setup code here, to run once:
  pinMode(PUL,OUTPUT);
  pinMode(DIR,OUTPUT);

  setAngle = -360.0;
}

void loop() {
  motorAngle = (motorStep / 800.0)*360;  
  
  error = setAngle - motorAngle;
  
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
