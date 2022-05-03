float pos;                      // Actuator Position
float conNum = 0.104167;         // Constant to convert ADC to mm 0.1042
                                // Equal to (960 (ADC at 100mm) - 44 (ADC at mm)/100mm)^-1


unsigned long currentTime, prevTime;
float val, controlSignal, analogCount, potenAnalogCount, prevError,cumulativeError, instanteneousError, errorRate;
int potenPos;

const int ALI = 5;
const int BLI = 6;

const int AHI = 10;
const int BHI = 11;

const int Disable = 9;


float kp = 2;
float ki = 0.015 ;
float kd = 0;
                                
void setup() {
  pinMode(A0, INPUT);         // Configure Analog In pin 0 as an Input

pinMode(ALI,OUTPUT);
pinMode(BLI,OUTPUT);
  
  pinMode(AHI, OUTPUT);        // Configure pin 10 as an Output
  pinMode(BHI, OUTPUT);      // Configure pin 11 as an Output

  pinMode(Disable, OUTPUT); 
  

  //val = 100;
Serial.begin(9600);
Serial.println("what position do you want the linear actuator to take?");
while(Serial.available()==0){}
val = Serial.parseFloat();
  
}


void loop() {


  
//Tracking the ADC 29 to 989//
analogCount = ((960*(val) + 29)/100);
potenAnalogCount = analogRead(A0);

//Time                        
 currentTime = millis();
 float elapsedTime = (float(currentTime - prevTime)/1e3); 

                          
/////PID Setup///////////////////////////////////
instanteneousError = analogCount - potenAnalogCount;        // Error
cumulativeError +=  instanteneousError*elapsedTime;  // integral action
errorRate = (instanteneousError - prevError)/elapsedTime; // derivative action

// PID OUTPUT
  float controlSignal = kp*instanteneousError + ki*cumulativeError + kd*errorRate;
 


  // store previous error
  prevError = instanteneousError; //store Instanteneous Error
  prevTime = currentTime; //store current time
 


                         //Driving the motor
                  float PWMval = fabs(controlSignal);
                                   if( PWMval > 200 ){
                                       PWMval = 200;
                                       }

                                       



// setting the motor direction
 int directn = 1;
 if(controlSignal<0){
   directn = -1;
 }



if(directn == 1){

analogWrite(BLI,PWMval);
analogWrite(ALI,0);
   // extend Actuator
    digitalWrite(AHI, HIGH);
    digitalWrite(BHI, HIGH);
    digitalWrite(Disable, LOW);
    //pos = readPotentiometer(); // Print position value to the Serial Display
   
 }
  else if(directn == -1){
analogWrite(BLI,0);
analogWrite(ALI,PWMval);
    // retract Actuator
    digitalWrite(AHI, HIGH);
    digitalWrite(BHI, HIGH);
    digitalWrite(Disable, LOW);
    //pos = readPotentiometer();
  
 }
  else{
analogWrite(ALI,0);
analogWrite(BLI,0);
    // Stop Actuator
    digitalWrite(AHI, HIGH);
  digitalWrite(BHI, HIGH);
  digitalWrite(Disable, LOW);
  }

  
 


 potenPos = 0.104167*(analogRead(A0) - 29); 
//potenPos = int (100 / 960)*(analogRead(A0) - 31);
  Serial.print(val);
  Serial.print(" ");
  Serial.print(potenPos);
  Serial.println();
}

/*Function to Read Potentiometer and Convert it to Inches*/
float readPotentiometer(void){
  float pos;
  pos = analogRead(A0);
 pos = conNum*(analogRead(A0) - 29); // 29 ADC is equal to 0mm
  return pos;
}
