#include "drv8871.h"
#include <SPI.h>
#include <LoRa.h> //https://github.com/sandeepmistry/arduino-LoRa

//Pins:
const int steeringFeedbackPinA = 4;
const int steeringFeedbackPinB = 5;
const int steeringFeedbackPinC = 6;
const int steeringDriverPin1 = 7;
const int steeringDriverPin2 = 8;
const int PWMmotorPin = 9;
const int PWMrssiPin = 10;


const int LoRaSSPin = A0; //output
const int LoRaResetPin = A1; //output
const int LoRaDioPin = A2; //input

//Constants:
const int failsafeTimeout = 250; //ms
const uint8_t failsafeVal = 0b01011011;
const unsigned int STOP_PWM_VAL = 127;

//Variables:
uint8_t rx=0;
int rssi=-150;
unsigned long nextFailsafeTimeout = 0;
int previousState = 0; //holds steering feedback position from last read.


//Objects:
DRV8871 steeringDriver(steeringDriverPin1,steeringDriverPin1);


void  setup()  {
  Serial.begin(115200);
  //while(!Serial);
  LoRa.setPins(LoRaSSPin, LoRaResetPin, LoRaDioPin);
  if (!LoRa.begin(433E6)) {
    Serial.println("Starting LoRa failed!");
    while (1);
  }
  /* PRESCALE DIVIDER FOR PWM FRQ ON PINS 9 and 10:
  16'000'000 / 256 / 2 = 31250Hz 
  0x01 1 31250
  0x02 8 3906.25
  0x03 64 488.28125
  0x04 256 122.0703125
  0x05 1024 30.517578125
  */
  TCCR1B = TCCR1B & 0b11111000 | 0x04; // <- Works.
  //TCCR1B = TCCR1B & 0b11111000 | 0x05;
  pinMode(steeringFeedbackPinA,INPUT_PULLUP);
  pinMode(steeringFeedbackPinB,INPUT_PULLUP);
  pinMode(steeringFeedbackPinC,INPUT_PULLUP);
  pinMode(PWMmotorPin,OUTPUT);
  pinMode(PWMrssiPin,OUTPUT);

  Serial.println("Good to go!");
  }
              
int rssi=-150;

void loop()  {
  if(LoRa.parsePacket()){
      rx = LoRa.read(); 
      rssi = LoRa.packetRssi();
      if(LoRa.available()) LoRa.flush(); //flush the RX buffer...                   
      //Serial.print("RX!: "); 
      //Serial.print(rx,BIN); 
      //Serial.print(", with RSSI: ");
      //Serial.print(rssi); 
      //Serial.print(','); 
      //Serial.print(millis()-(nextFailsafeTimeout-failsafeTimeout)); //milliseconds since last RX
      //Serial.println();
      nextFailsafeTimeout = millis() + failsafeTimeout;
    }
  if(millis() > nextFailsafeTimeout) rx = failsafeVal;
  
  //Report RSSI over PWM to OSD :D
  analogWrite(PWMrssiPin,map(rssi,-150,20,0,255));

  //Parse received data:
  int throttle = -3+(rx&0b00000111);
  int steering = (-3+((rx&0b00111000)>>3))*(-1); //-1 reverses steering direction.
  uint8_t speedFactor = (rx&0b11000000)>>6;
  
  //Handle speed:
                      //    1-3                -3-3
  int throttlePWMdiff = speedFactor * 14 * throttle; // 127/(3*3) = 14.11
  uint8_t pwmVal = STOP_PWM_VAL + throttlePWMdiff;
  int mapVal = map(pwmVal,0,255,32,62);
  analogWrite(PWMmotorPin, mapVal);


  //Handle steering:
  previousState = readSteeringFeedback();
  int steeringDelta = steering-previousState; 
  if(steeringDelta<0) steeringDriver.forward();//need to go left
  else if(steeringDelta>0) steeringDriver.reverse();//need to go right
  else steeringDriver.brake(); //need to go nowhere  
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