#define portOfPin(P)\
  (((P)>=0&&(P)<8)?&PORTD:(((P)>7&&(P)<14)?&PORTB:&PORTC))
#define ddrOfPin(P)\
  (((P)>=0&&(P)<8)?&DDRD:(((P)>7&&(P)<14)?&DDRB:&DDRC))
#define pinOfPin(P)\
  (((P)>=0&&(P)<8)?&PIND:(((P)>7&&(P)<14)?&PINB:&PINC))
#define pinIndex(P)((uint8_t)(P>13?P-14:P&7))
#define pinMask(P)((uint8_t)(1<<pinIndex(P)))

#define pinAsInput(P) *(ddrOfPin(P))&=~pinMask(P)
#define pinAsInputPullUp(P) *(ddrOfPin(P))&=~pinMask(P);digitalHigh(P)
#define pinAsOutput(P) *(ddrOfPin(P))|=pinMask(P)
#define digitalLow(P) *(portOfPin(P))&=~pinMask(P)
#define digitalHigh(P) *(portOfPin(P))|=pinMask(P)
#define isHigh(P)((*(pinOfPin(P))& pinMask(P))>0)
#define isLow(P)((*(pinOfPin(P))& pinMask(P))==0)
#define digitalState(P)((uint8_t)isHigh(P))



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
  pinAsInputPullUp(inputCLK_1);
  pinAsInputPullUp(inputDT_1);

  previousStateCLK_1 = digitalState(inputCLK_1);
  Serial.begin(9600);
}

void loop() {
  // put your main code here, to run repeatedly:
  read_encdr_1();
  
  t = micros();
  if ((t-t_old) > freq)
  {
    Serial.println(counter_1);
    t_old = t;
  }
}

void read_encdr_1()
{
  currentStateCLK_1 = digitalState(inputCLK_1);

  // Read encoder 1
  if (currentStateCLK_1 != previousStateCLK_1){ 
    // If the inputDT state is different than the inputCLK state then 
    // the encoder is rotating counterclockwise
    if (digitalState(inputDT_1) != currentStateCLK_1)
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
