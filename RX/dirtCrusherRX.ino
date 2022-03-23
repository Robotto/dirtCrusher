#include "drv8871.h"

const uint8_t failsafeVal = 0b01011011;
const int ADCpin = A3;
const int steeringFeedbackPinA = 4;
const int steeringFeedbackPinB = 5;
const int steeringFeedbackPinC = 6;
int previousState = 0; //holds steering feedback position from last read.
uint8_t batt=255;

DRV8871 steeringDriver(7,8);

//PWM STUFF:
const int motorPin = 9;
const unsigned int STOP_PWM_VAL = 127;

void setup() {
  Serial1.begin(1200);
  Serial.begin(115200);
  pinMode(steeringFeedbackPinA,INPUT_PULLUP);
  pinMode(steeringFeedbackPinB,INPUT_PULLUP);
  pinMode(steeringFeedbackPinC,INPUT_PULLUP);
  pinMode(motorPin,OUTPUT);

  /* PRESCALE DIVIDER FOR PWM FRQ ON PINS 9 and 10:
  16'000'000 / 256 / 2 = 31250Hz 
            0x01 1 31250
            0x02 8 3906.25
            0x03 64 488.28125
            0x04 256 122.0703125
            0x05 1024 30.517578125
*/


  TCCR1B = TCCR1B & 0b11111000 | 0x04;
  analogWrite(motorPin,STOP_PWM_VAL);

}

uint8_t rx;
unsigned long nextFailsafeTimeout = 0;
int failsafeTimeout = 500; //ms

void loop(){
  if (Serial1.available()) {
    rx=Serial1.read();
    Serial1.write(batt);
    nextFailsafeTimeout = millis() + failsafeTimeout;
  }
  if(millis() > nextFailsafeTimeout) rx = failsafeVal;
  
  //Parse received data:
  int throttle = -3+(rx&0b00000111);
  int steering = (-3+((rx&0b00111000)>>3))*(-1); //-1 reverses steering direction.
  uint8_t speedFactor = (rx&0b11000000)>>6;

                      //    1-3                -3-3
  int throttlePWMdiff = speedFactor * 14 * throttle; // 127/(3*3) = 14.11
  uint8_t pwmVal = STOP_PWM_VAL + throttlePWMdiff;
  int mapVal = map(pwmVal,0,255,31,62);
  analogWrite(motorPin, mapVal);
  
  int steeringDelta = steering-readSteeringFeedback(); 
  //TODO: check direction of steering driver vs left/right
  if(steeringDelta<0) steeringDriver.forward();//need to go left
  else if(steeringDelta>0) steeringDriver.reverse();//need to go right
  else steeringDriver.brak(e(); //need to go nowhere
  

  batt=readBatt();

// if(rx != failsafeVal) Serial.print("RX: "); Serial.print(rx,BIN); Serial.print(" -> throttle="); Serial.print(throttle); Serial.print(", steering="); Serial.print(steering); Serial.print(", SpeedFactor="); Serial.print(speedFactor); Serial.print(", pwmVal="); Serial.print(pwmVal); Serial.print(", SteeringDelta="); Serial.print(steeringDelta); Serial.print(", Batt:"); Serial.println(batt);

//  Serial.print(pwmVal);Serial.print(',');Serial.println(mapVal);

}

int readSteeringFeedback(){ //negative is turning left!
  int aState = digitalRead(steeringFeedbackPinA);
  int bState = digitalRead(steeringFeedbackPinB);
  int cState = digitalRead(steeringFeedbackPinC);

  if( aState & bState & cState ) return previousState; //non-discrete in-between-state with all pins high - Keep moving the steering.
  //Now we know the feedback is in a discrete state:
  if( bState ) return 2-cState+aState; //Steering feedback is positive (1,2,3)
  else if ( cState & !aState ) return 0; // A=0, B=0; C=1
  else return -3+aState+cState; //-3,-2,-1
}

//Voltage divider: VBATT -> 51K <-ADC-> 75K -> GND
#define V_BATTMAX 8.4
#define V_BATTMIN 7.0
#define V_ADCMAX 5.0
#define ADCMAX 1023.0
#define V_DIVIDER_MAX_OUT 5.0
#define DIVIDER_FACTOR 0.595     //Factor = 1-(R1/(R1+R2)) = V_BATTMAX / V_DIVIDER_MAX_OUT
#define V_CALIBATED_OFFSET 0.06
#define N_MEASUREMENTS 16

/*
ADC takes about 116uS pr sample:
Nsamples:	Duration [uS]:  Shifts:
16	      1856            4
32	      3712            5
64	      7424            6
128	      14848           7
*/

/*
uint8_t readBatt(){
  //Do a bunch of ADC measurements and convert them to average Vbatt, append Vbatt to report
  unsigned long ADCSum = 0;
  for (int i=0; i<N_MEASUREMENTS; i++) ADCSum+=analogRead(ADCpin); 
  unsigned int ADCavg = ADCSum/N_MEASUREMENTS;
  float vBatt = (float)ADCavg*V_ADCMAX/ADCMAX/DIVIDER_FACTOR+V_CALIBATED_OFFSET; 
  return uint8_t(((vBatt - V_BATTMIN) * 100.0 / (V_BATTMAX - V_BATTMIN))); //calculate battery percentage
}
*/
// 7.0V -> 213 , 8.4V->255
uint8_t readBatt(){
  unsigned long ADCSum = 0;
  for (int i=0; i<N_MEASUREMENTS; i++) ADCSum+=analogRead(ADCpin); 
  return (uint8_t) map(ADCSum>>6,213,255,0,100);  //shift down from N_MEASUREMENTS, and also divide by 4 to fit data in one byte
}