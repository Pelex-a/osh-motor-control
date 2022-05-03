//    Encoder Pins
const int encoderPhaseA = 2; // Phase A of the rotary encoder
const int encoderPhaseB = 3; // Phase B of the rotary encoder

// Motor Driver Pins
const int pwmPin = 6; //Enable PWM pin
const int input1 = 10; //Direction pin
const int input2 = 11; // Direction pin

// Global Variables
unsigned long prevTime, currentTime;
int prevCount, target, variable,counter;
volatile int interruptCount;
float instanteneousError, vfiltered, vel_previous, errorRate, cumulativeError, prevError, targetVelocity;

  
void setup() {

// start serial port at 9600 bps
  Serial.begin(9600);
//Configuring the Pins
  pinMode(encoderPhaseA,INPUT);
  pinMode(encoderPhaseB,INPUT);
  pinMode(pwmPin,OUTPUT);
  pinMode(input1,OUTPUT);
  pinMode(input2,OUTPUT);

Serial.println("Which variable do you want to control?" "\n" "1--POSITION" "\n" "2--SPEED ");
while(Serial.available()==0){}
variable = Serial.parseInt();
while(Serial.available()!=0){Serial.parseInt();}

if (variable == 1){ 
Serial.println ("Enter Target");

while(Serial.available()==0){};
target = Serial.parseInt();
while(Serial.available()!=0){Serial.parseInt();}
}

else if (variable == 2){ Serial.println ("Enter Desired Speed");
while(Serial.available()==0){}
targetVelocity = Serial.parseFloat();
while(Serial.available()!=0){Serial.parseFloat();}
}

else{ Serial.println ("ERROR");}

attachInterrupt(digitalPinToInterrupt(encoderPhaseA),
                  Encoder,RISING);

}
void loop() {

  if (variable == 1){

    //Call the position controller
    Posfunction(); 
  }
 else if(variable == 2)
 {
  //Call the velocity controller
  Velfunction();
  }
  else {//Do nothing!
    }
}

void Posfunction(){
 
// computing time
  currentTime = micros();
  float elapsedTime = ((float) (currentTime - prevTime)* 1e-6);
  //prevTime = currentTime;

//PID Constants
  double kp = 8;
  double kd = 0.00;  //25;
  double ki = 0.0 ; //5;

//setting up the PID controller
  instanteneousError = interruptCount - target;  // Propotional action

  cumulativeError +=  instanteneousError*elapsedTime;  // integral action

  errorRate = (instanteneousError - prevError)/elapsedTime; // derivative action

// PID OUTPUT
  float controlOutput = kp*instanteneousError + ki*cumulativeError + kd*errorRate; //control signal

  
// store previous error
  prevError = instanteneousError; //store Instanteneous Error
  prevTime = currentTime; //store current time


  // The speed/velocity of the motor
  float PWMValue = fabs(controlOutput);
  if( PWMValue > 255 ){
    PWMValue = 255;
    }

    // setting the motor direction
  int directn = 1;
  if(controlOutput<0){
    directn = -1;
  }

analogWrite(pwmPin,PWMValue);
  if(directn == 1){
    digitalWrite(input2,HIGH);
    digitalWrite(input1,LOW);
    Serial.println(+1);
  }
  else if(directn == -1){
    digitalWrite(input2,LOW);
    digitalWrite(input1,HIGH);
    Serial.println(-1);
  }
  else{
    digitalWrite(input2,LOW);
    digitalWrite(input1,LOW);
  }  
  
Serial.print(target);
  Serial.print(" ");
  Serial.print(interruptCount);
  Serial.println();
}



void Velfunction(){
 
  int count = 0;
 
  
  noInterrupts();
    count = interruptCount;
   
interrupts();

  // Computing time
  currentTime = micros();
  float elapsedTime = ((float) (currentTime-prevTime))/1.0e6;
  float velo = (count - prevCount)/elapsedTime;
  prevCount = count;
  prevTime = currentTime;

  // Convert count/s to RPM
  float vel_RPM = velo/600.0*60.0;


  // filter (25 Hz)
  vfiltered = 0.854*vfiltered + 0.0728*vel_RPM + 0.0728*vel_previous;
  vel_previous = vel_RPM;
 



  // PID constants
  float kp = 5;
  float ki = 1;

  //computing PID Controller//////////////////////////////
  instanteneousError = targetVelocity-vfiltered;
  cumulativeError += instanteneousError*elapsedTime;
  float controlOutput = kp*instanteneousError + ki*cumulativeError;
  //////////////////////////////////////////////////////////
  
  // Setting the PWM value and direction
  int dir = 1;
  if (controlOutput<0){
    dir = -1;
  }
  int pwmLevel = (int) fabs(controlOutput);
  if(pwmLevel > 255){
    pwmLevel = 255;
  }

  
  analogWrite(pwmPin,pwmLevel); // Motor speed
  if(dir == 1){ 
    // Turn one way
    digitalWrite(input1,HIGH);
    digitalWrite(input2,LOW);
  }
  else if(dir == -1){
    // Turn the other way
    digitalWrite(input1,LOW);
    digitalWrite(input2,HIGH);
  }
  else{
    // Or dont turn
    digitalWrite(input1,LOW);
    digitalWrite(input2,LOW);    
  }

  Serial.print(targetVelocity);
  Serial.print(" ");
  Serial.print(vfiltered);
  Serial.println();
  delay(1);
}

void Encoder(){
  // Read encoder B when encoderPhaseA rises
  int b = digitalRead(encoderPhaseB);
  
  if(b>0){
    // If B is high, increment forward
    counter = 1;
  }
  else{
    // Otherwise, increment backward
    counter = -1;
  }
  interruptCount += counter;
}
