#include <SPI.h>
#include <LoRa.h> //https://github.com/sandeepmistry/arduino-LoRa

//SX1278 pins:
const int LoRaSSPin = A0;
const int LoRaResetPin = A1;
const int LoRaDioPin = A2; //unused

//Dirtcrusher remote controller pins
const int xPin = 8; //ORANGE 
const int yPin = 5; //GREEN
const int throttle_aPin = A3; //yellow
const int throttle_bPin = 10; //WHITE
const int steering_aPin = 7; //yellow
const int steering_bPin = 4; //white
const int fasterPaddlePin = 9; //pink 
const int slowerPaddlePin = 6; //blue


#define TXPERIOD 25UL //30mS -> 33Hz / 25mS -> 40Hz

/*
Payload byte layout:
0bxxxxxxxx
  ||||||||
  ||||||| \_ Throttle STICK BIT #0
  |||||| \__ Throttle STICK BIT #1
  ||||| \___ Throttle STICK BIT #2
  |||| \____ Steering STICK BIT #0
  ||| \_____ Steering STICK BIT #1
  || \______ Steering STICK BIT #2
  | \_______ SPEED BIT 1
  \________  SPEED BIT 2
*/

void setup() {
  Serial.begin(115200);

  LoRa.setPins(LoRaSSPin, LoRaResetPin, LoRaDioPin);
  
  if (!LoRa.begin(433E6)) {
    Serial.println("Starting LoRa failed!");
    while (1);
  }
  //READ THIS: https://forum.arduino.cc/t/what-is-lora/595381
  //LoRa.enableCrc();
  LoRa.setSpreadingFactor(9); //https://github.com/sandeepmistry/arduino-LoRa/blob/master/API.md#spreading-factor
  LoRa.setSignalBandwidth(125E3); //default: 125E3  - https://github.com/sandeepmistry/arduino-LoRa/blob/master/API.md#signal-bandwidth
  //void LoRaClass::setGain(uint8_t gain) //default: AGC
  

  //Controller pins
  digitalWrite(xPin,LOW);
  digitalWrite(yPin,LOW);
  pinMode(xPin,INPUT);
  pinMode(yPin,INPUT);
  pinMode(throttle_aPin,INPUT_PULLUP);
  pinMode(throttle_bPin,INPUT_PULLUP);
  pinMode(steering_aPin,INPUT_PULLUP);
  pinMode(steering_bPin,INPUT_PULLUP);
  pinMode(fasterPaddlePin,INPUT_PULLUP);
  pinMode(slowerPaddlePin,INPUT_PULLUP);  
}

void loop() {
  static uint8_t lastPayload = 0;
  static unsigned long lastTXtime = 0;
  
  uint8_t speed = checkPaddles();
  uint8_t throttleVal = 3 + getThrottle(); //-3 to 3 -> 0 to 6
  uint8_t steeringVal = 3 + getSteering();

  uint8_t payload = speed << 6 | steeringVal << 3 | throttleVal; ///JOIN THE VALUES into one byte
   
  if(millis()-lastTXtime > TXPERIOD)// || payload != lastPayload) 
  { 
   if (LoRa.beginPacket()) //If radio is ready to transmit
        {
            LoRa.write(payload);
            LoRa.endPacket(true); //Async: don't wait for TX confirmation
            //Serial.print("dt:"); Serial.println(millis()-lastTXtime);
            /*
            Serial.print("speed:"); Serial.print(speed); Serial.print(",");
            Serial.print("Throttle:"); Serial.print(throttleVal); Serial.print(",");
            Serial.print("Steering:"); Serial.print(steeringVal); Serial.print(",");
            Serial.print("Binary:"); Serial.print(payload,BIN); Serial.println();
            */
            lastTXtime = millis();
            lastPayload = payload;
        }
    //else Serial.println("Not ready to transmit...");
  }
}

int getThrottle() { //returns -3 -> +3
  return readStick(xPin, yPin, throttle_aPin, throttle_bPin);
}

int getSteering() { //returns -3 -> +3
  return readStick(xPin, yPin, steering_aPin, steering_bPin);
}

//WORST CASE RUNTIME: >20mS
uint8_t checkPaddles()  {
  static uint8_t speed = 1;
  static bool lastState_fasterPaddle = HIGH;
  static bool lastState_slowerPaddle = HIGH;

  if(!digitalRead(fasterPaddlePin) || !digitalRead(slowerPaddlePin) ) delay(20);

  if(!digitalRead(fasterPaddlePin) && lastState_fasterPaddle) { //paddle is pressed 
    lastState_fasterPaddle = LOW; //set last state to pressed
    speed++;
  }

  if(!digitalRead(slowerPaddlePin) && lastState_slowerPaddle) { //paddle is pressed 
    lastState_slowerPaddle = LOW; //set last state to pressed
    speed--;
  }
  if(speed>3) speed=3;
  else if(speed<1) speed=1;

  if(digitalRead(fasterPaddlePin)) lastState_fasterPaddle = HIGH; //reset last state  
  if(digitalRead(slowerPaddlePin)) lastState_slowerPaddle = HIGH; //reset last state

  return speed;
}

int readStick(int X, int Y, int A, int B) {

  pinMode(X,OUTPUT);
  //digitalWrite(X,LOW); //perhaps redundant if state never changes?
  int aState = digitalRead(A);
  int bState = digitalRead(B);
  pinMode(X,INPUT);

  if( !aState ) return 1+(1-bState); //A=low (state 1 or 2)
  else if ( !bState ) return 3; //A=high, B=low

  pinMode(Y,OUTPUT);
  //digitalWrite(Y,LOW); //perhaps redundant if state never changes?
  aState = digitalRead(A);
  bState = digitalRead(B);
  pinMode(Y,INPUT);

  if( !aState ) return -1*(1+(1-bState)); //A=low (state -1 or -2)
  else if ( !bState ) return -3; //A=high, B=low

  return 0;
}
