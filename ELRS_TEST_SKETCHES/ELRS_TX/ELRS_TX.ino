#include <Arduino.h>

#include "config.h"
#include "crsf.h"

int Throttle_value = 0;
int Rudder_value = 0;

int loopCount = 0; // for ELRS seeting


//Dirtcrusher remote controller pins
const int xPin = 8; //ORANGE 
const int yPin = 5; //GREEN
const int throttle_aPin = A3; //yellow
const int throttle_bPin = 10; //WHITE
const int steering_aPin = 7; //yellow
const int steering_bPin = 4; //white
const int fasterPaddlePin = 9; //pink 
const int slowerPaddlePin = 6; //blue


float batteryVoltage;

int currentPktRate = 0;
int currentPower = 0;
int currentDynamic = 0;
int currentSetting = 0;


uint32_t currentMillis = 0;

uint8_t crsfPacket[CRSF_PACKET_SIZE];
uint8_t crsfCmdPacket[CRSF_CMD_PACKET_SIZE];
int16_t rcChannels[CRSF_MAX_CHANNEL];
uint32_t crsfTime = 0;

CRSF crsfClass;

float RSSI_WORST=108.0; //dBm (negative)
float RSSI_BEST=50.0;  //dBm (negative)
float RSSI_PERCENT=0;


void setup()
{
    // inialize rc data
    for (uint8_t i = 0; i < CRSF_MAX_CHANNEL; i++) {
        rcChannels[i] = CRSF_CHANNEL_VALUE_MIN;
    }

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

    Serial.begin(250000);
    crsfClass.begin(); //Set to use Serial1 on the Leonardo...

}

void loop()
{
    uint16_t receiverBatteryVoltage=0;

    uint32_t currentMicros = micros();
    crsfClass.update();

    RSSI_PERCENT=0;
    if(crsfClass.linkUP())
{
      const crsf_sensor_battery_t* batt = crsfClass.getBatt();
      receiverBatteryVoltage = batt->voltage;
      const crsfLinkStatistics_t* stat_ptr = crsfClass.getLinkStatistics();
      uint8_t RSSI = stat_ptr->downlink_RSSI;
      RSSI_PERCENT = map(RSSI,RSSI_WORST,RSSI_BEST,0,100);
      RSSI_PERCENT = constrain(RSSI_PERCENT, 0, 100);
}   


  int8_t speedFactor = checkPaddles(); //1, 2 or 3
  int8_t throttleInput = getThrottle(); //-3 to 3
  int8_t steeringInput = getSteering(); //-3 to 3
    

                     //    1-3              -3-3
  int throttleDiff = speedFactor * 91 * throttleInput; // 91 = (CRSF_CHANNEL_VALUE_SPAN/2)/(3*3)
  
  int throttleVal = CRSF_CHANNEL_VALUE_MID + throttleDiff;

  int rudderDiff = 273*steeringInput; //273 = ((CRSF_CHANNEL_VALUE_SPAN/2)/3)
  int rudderVal = CRSF_CHANNEL_VALUE_MID + rudderDiff;
  //TEST waves:
    //Throttle_value = (uint16_t)(((1.0+sin((float)millis()/1000.0))*0.5)*ADC_MAX);
    //Rudder_value = (uint16_t)(((1.0+cos((float)millis()/1000.0))*0.5)*ADC_MAX); 

    //rcChannels[AILERON]   = 0; 
    rcChannels[THROTTLE]  = constrain(throttleVal,CRSF_CHANNEL_VALUE_MIN,CRSF_CHANNEL_VALUE_MAX);
    rcChannels[RUDDER]    = constrain(rudderVal,CRSF_CHANNEL_VALUE_MIN,CRSF_CHANNEL_VALUE_MAX);

/*
    Serial.print("speedFactor:");
    Serial.print(speedFactor);
    */
    Serial.print("throttleInput:");
    Serial.print(throttleInput);
/*
    Serial.print("throttleDiff:");
    Serial.print(throttleDiff);
    Serial.print("throttleVal:");
    Serial.print(throttleVal);

  */  
    Serial.print("steeringInput:");
    Serial.print(steeringInput);
    //Serial.print(",STEERING:");
    //Serial.print(rcChannels[RUDDER]);
    Serial.print(",BATTERY:");
    Serial.print(receiverBatteryVoltage/10);
    Serial.print(",TELEMETRY_RSSI%/10:");
    Serial.print(RSSI_PERCENT/10);
    Serial.print(",MIN:");
    Serial.print(-3);
    Serial.print(",MAX:");
    Serial.println(9);
  


    if (currentMicros > crsfTime) {

            if (loopCount <= 500) { // repeat 500 packets to build connection to TX module
                // Build commond packet
                crsfClass.crsfPrepareDataPacket(crsfPacket, rcChannels);
                crsfClass.CrsfWritePacket(crsfPacket, CRSF_PACKET_SIZE);
                loopCount++;
            }

                //TODO: Is this at all neccesary??

            if (loopCount > 500 && loopCount <= 505) { // repeat 5 packets to avoid bad packet, change rate setting
                // Build commond packet
                if (currentSetting == 1 || currentSetting == 2) {
                    crsfClass.crsfPrepareCmdPacket(crsfCmdPacket, ELRS_PKT_RATE_COMMAND, currentPktRate);
                    crsfClass.CrsfWritePacket(crsfCmdPacket, CRSF_CMD_PACKET_SIZE);
                } else if (currentSetting == 3) {
                    crsfClass.crsfPrepareCmdPacket(crsfCmdPacket, ELRS_BIND_COMMAND, ELRS_START_COMMAND);
                    crsfClass.CrsfWritePacket(crsfCmdPacket, CRSF_CMD_PACKET_SIZE);
                } else if (currentSetting == 4) {
                    crsfClass.crsfPrepareCmdPacket(crsfCmdPacket, ELRS_WIFI_COMMAND, ELRS_START_COMMAND);
                    crsfClass.CrsfWritePacket(crsfCmdPacket, CRSF_CMD_PACKET_SIZE);
                }
                loopCount++;
            } else if (loopCount > 505 && loopCount <= 510) { // repeat 5 packets to avoid bad packet, change TX power level
                if (currentSetting == 1 || currentSetting == 2) {
                    crsfClass.crsfPrepareCmdPacket(crsfCmdPacket, ELRS_POWER_COMMAND, currentPower);
                    crsfClass.CrsfWritePacket(crsfCmdPacket, CRSF_CMD_PACKET_SIZE);
                }
                loopCount++;
            } else if (loopCount > 510 && loopCount <= 515) { // repeat 5 packets to avoid bad packet, change TX dynamic power setting
                if (currentSetting == 1 || currentSetting == 2) {
                    crsfClass.crsfPrepareCmdPacket(crsfCmdPacket, ELRS_DYNAMIC_POWER_COMMAND, currentDynamic);
                    crsfClass.CrsfWritePacket(crsfCmdPacket, CRSF_CMD_PACKET_SIZE);
                }
                loopCount++;
            } else {
                crsfClass.crsfPrepareDataPacket(crsfPacket, rcChannels);
                crsfClass.CrsfWritePacket(crsfPacket, CRSF_PACKET_SIZE);
            }

        crsfTime = currentMicros + CRSF_TIME_BETWEEN_FRAMES_US;
    }
}


int getThrottle() { //returns -3 -> +3
  return readStick(xPin, yPin, throttle_aPin, throttle_bPin);
}

int getSteering() { //returns -3 -> +3
  return readStick(xPin, yPin, steering_aPin, steering_bPin);
}

//WORST CASE RUNTIME: >5mS
uint8_t checkPaddles()  {
  static uint8_t speed = 1;
  static bool lastState_fasterPaddle = HIGH;
  static bool lastState_slowerPaddle = HIGH;

  if(!digitalRead(fasterPaddlePin) || !digitalRead(slowerPaddlePin) ) delay(5);

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