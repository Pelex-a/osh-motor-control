
const int pwmPin = 6;
const int IN1 = 10;
const int IN2 = 11;
int dutyCycle = 100;


void setup() {
 

  Serial.begin(9600);
  
  pinMode(pwmPin,OUTPUT);
  pinMode(IN1,OUTPUT);
  pinMode(IN2,OUTPUT);
}

  void loop() {
 
analogWrite(pwmPin,dutyCycle);


   // backward or clockwise
    digitalWrite(IN2,HIGH);
    digitalWrite(IN1,LOW);
    Serial.print(dutyCycle);
   
 

    
  }  
  
