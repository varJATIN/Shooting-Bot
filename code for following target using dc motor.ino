// PID motor position control.

// Thanks to Brett Beauregard for his nice PID library http://brettbeauregard.com/blog/2011/04/improving-the-beginners-pid-introduction/
//for dc motor position control according to setpoint .
//setpoint to be recieved from python code for image detection via serial transmittor pins in arduino.
//optical orange encoder 600ppm is used.
#include <PinChangeInt.h>
#include <PID_v1.h>
#define encodPinA1      2                       // Quadrature encoder A pin
#define encodPinB1      3                      // Quadrature encoder B pin
#define M1              9                       // PWM outputs to L298N H-Bridge motor driver module
#define M2              10 
#define M3              12  
                                                               
double kp = 4.8 , ki =0.097 , kd =0.02       ;      // modify for optimal performance
double input = 0, output = 0, setpoint = 0;
long temp;
signed volatile long counter = 0;
PID myPID(&input, &output, &setpoint, kp, ki, kd, DIRECT);  // if motor will only run at full speed try 'REVERSE' instead of 'DIRECT'

void setup() {
pinMode(encodPinA1, INPUT_PULLUP);                  // quadrature encoder input A
pinMode(encodPinB1, INPUT_PULLUP);
digitalWrite(encodPinA1,HIGH);
digitalWrite(encodPinB1,HIGH);// quadrature encoder input B
attachInterrupt(0, ai0, RISING);
 //B rising pulse from encodenren activated ai1(). AttachInterrupt 1 is DigitalPin nr 3 on moust Arduino.
attachInterrupt(1, ai1, RISING);               // update encoder position
//TCCR1B = TCCR1B & 0b11111000 | 1;                   // set 31KHz PWM to prevent motor noise
myPID.SetMode(AUTOMATIC);
myPID.SetSampleTime(1);
myPID.SetOutputLimits(-255, 255);
Serial.begin (9600);                              // for debugging
setpoint=60;
}

int z=0;
signed int x,y;
void loop() {
  x=counter/3.33;
  y=x%360;
  input = y;                                // data from encoder
  Serial.println(y); 
  
  if((abs(setpoint-y))>5)
  {myPID.Compute();                                    // calculate new output
  pwmOut(output); 
    }
   else{
   pwmOut(z);
   digitalWrite(M2,LOW);
   digitalWrite(M3,LOW); 
    delay(200);
    }
    // monitor motor position
  //setpoint = analogRead(0) * 5;                       // modify to fit motor and encoder characteristics, potmeter connected to A0
                                      // drive L298N H-Bridge module
 // Serial.print("error");
  //Serial.println(input);
delay(20);}



void pwmOut(int out) {                                // to H-Bridge board
    if (out > 0) {
    analogWrite(M1, out);                             // drive motor CW
    analogWrite(M2, 150);
    digitalWrite(M3,LOW);
  }
  else {
    analogWrite(M1,abs(out));
    digitalWrite(M2,LOW);                        // drive motor CCW
    analogWrite(M3,150);
}
}
void ai0() {
  // ai0 is activated if DigitalPin nr 2 is going from LOW to HIGH
  // Check pin 3 to determine the direction
if(digitalRead(3)==LOW) {
    counter++;
  }else{
    counter--;
  }
}
void ai1() {
  // ai0 is activated if DigitalPin nr 3 is going from LOW to HIGH
  // Check with pin 2 to determine the direction
 if(digitalRead(2)==LOW) {
  counter--;
  }else{
  counter++;
  }

}
