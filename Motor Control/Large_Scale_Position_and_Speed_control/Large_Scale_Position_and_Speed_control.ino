//    Encoder Pins
const int encoderPhaseA = 2; // Phase A of the rotary encoder
const int encoderPhaseB = 3; // Phase B of the rotary encoder

// Motor Driver Pins

const int ALI = 5; //Enable PWM pin
const int BLI = 6; //Enable PWM pin

const int AHI = 10; // Direction pin
const int BHI = 11; // Direction pin

const int Disable = 9;


// Global Variables
unsigned long prevTime, currentTime;
int prevCount, target, variable,counter;
volatile int interruptCount;
float instanteneousError, filteredVelocity, prevVelocity, errorRate, cumulativeError, prevError, targetVelocity;

  
void setup() {

// start serial port at 9600 bps
  Serial.begin(9600);
//Configuring the Pins

Serial.begin(9600);
pinMode(encoderPhaseA,INPUT_PULLUP); // INPUT can be used if the encoder has power leads
pinMode(encoderPhaseB,INPUT_PULLUP);
  
pinMode(ALI,OUTPUT);
pinMode(BLI,OUTPUT);
  
pinMode(AHI, OUTPUT);        // Configure pin 10 as an Output
pinMode(BHI, OUTPUT);      // Configure pin 11 as an Output

pinMode(Disable, OUTPUT);
 


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
 
// compute time
  currentTime = millis();
  float elapsedTime = ((float) (currentTime - prevTime)* 1e-3); //converting from ms to s
  
  //setting up the PID controller

instanteneousError = target; //- interruptCount;  // Propotional action

cumulativeError +=  instanteneousError*elapsedTime;  // integral action

errorRate = (instanteneousError - prevError)/elapsedTime; // derivative action

//PID Constants
  double kp = 7;
  double kd = 0.025;
  double ki = 0;


// controller output
  float controlOutput = kp*instanteneousError + ki*cumulativeError + kd*errorRate;

 // store previous error
 prevError = instanteneousError; //store Instanteneous Error
 prevTime = currentTime; //store current time

// The speed/velocity of the motor
  float PWM_value  = fabs(controlOutput);
  if( PWM_value > 230 ){
    PWM_value = 230;
    }

    // setting the motor direction
  int directn = 1;
  if(controlOutput<0){
    directn = -1;
  }


if(directn == 1){
   // Forward Direction/anticlockwise
   analogWrite(BLI, PWM_value);
   analogWrite(ALI, 0);
    digitalWrite(AHI, HIGH);
    digitalWrite(BHI, HIGH);
    digitalWrite(Disable, LOW);  //Not Disabled
   
 }
  else if(directn == -1){
    //Backward Direction/clockwise
   analogWrite(ALI, PWM_value);
   analogWrite(BLI, 0);
    digitalWrite(AHI, HIGH);
    digitalWrite(BHI, HIGH);
    digitalWrite(Disable, LOW);  //Not Disabled
    
 }
  else{
    // Stop Actuator
       analogWrite(ALI, 0);
       analogWrite(BLI, 0);
    digitalWrite(AHI, HIGH);
  digitalWrite(BHI, HIGH);
  digitalWrite(Disable, LOW);




Serial.print(target);
  Serial.print(" ");
  Serial.print(interruptCount);
  Serial.println();
}
}


void Velfunction(){

int count = 0;
  noInterrupts();
    count = interruptCount;
interrupts();

   //Compute velocity using count per second
  currentTime = micros();
  float elapsedTime = ((float) (currentTime-prevTime))/1.0e6;
  float velocity = (count - prevCount)/elapsedTime; //in count/s
  prevCount = count;
  prevTime = currentTime;

//Convert count/s to RPM
  
  float velocity_RPM = velocity/4*60.0; //divided by encoder counts per revolution. Please check the encoder specifications for this.

// Applying low pass filter (25 Hz cutoff)
  filteredVelocity = 0.854*filteredVelocity + 0.0728*velocity_RPM + 0.0728*prevVelocity;
  prevVelocity = velocity_RPM;
 
 // Constants
  float kp = 15;
 float ki = 25;
 float kd = 1;

 // Computing PID
 instanteneousError = targetVelocity-filteredVelocity;   // proportional action
 cumulativeError += instanteneousError*elapsedTime;     // integral action
 errorRate = (instanteneousError - prevError)/elapsedTime; // derivative action
  
 float controlOutput = kp*instanteneousError + ki*cumulativeError;

 prevError = instanteneousError; //store Instanteneous Error

  // Set the motor speed and direction
  int directn = 1;
  if (controlOutput<0){
    directn = -1;
  }
  int PWM_value = (int) fabs(controlOutput);
  if(PWM_value > 230){
    PWM_value = 230; 
  }


if(directn == 1){
   // Forward Direction/anticlockwise
   analogWrite(BLI, PWM_value);
   analogWrite(ALI, 0);
    digitalWrite(AHI, HIGH);
    digitalWrite(BHI, HIGH);
    digitalWrite(Disable, LOW);  //Not Disabled
   
 }
  else if(directn == -1){
    //Backward Direction/clockwise
   analogWrite(ALI, PWM_value);
   analogWrite(BLI, 0);
    digitalWrite(AHI, HIGH);
    digitalWrite(BHI, HIGH);
    digitalWrite(Disable, LOW);  //Not Disabled
    
 }
  else{
    // Stop Actuator
       analogWrite(ALI, 0);
       analogWrite(BLI, 0);
    digitalWrite(AHI, HIGH);
  digitalWrite(BHI, HIGH);
  digitalWrite(Disable, LOW);
  }


 Serial.print(targetVelocity);
  Serial.print(" ");
  Serial.print(filteredVelocity);
  Serial.println();


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
