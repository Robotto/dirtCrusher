#include <Arduino.h>

#include "config.h"
#include "crsf.h"
#include <U8g2lib.h>
U8G2_SSD1306_128X32_UNIVISION_F_HW_I2C u8g2(U8G2_R0); 

#include "gfx.h"

int Throttle_value = 0;
int Rudder_value = 0;

int loopCount = 0; // for ELRS seeting

int VbattPin=A2;


//Dirtcrusher remote controller pins
const int xPin = 8; //ORANGE 
const int yPin = 5; //GREEN
const int throttle_aPin = A3; //yellow
const int throttle_bPin = 10; //WHITE
const int steering_aPin = 7; //yellow
const int steering_bPin = 4; //white
const int fasterPaddlePin = 9; //pink 
const int slowerPaddlePin = 6; //blue

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


  u8g2.begin();
//  u8g2.setDisplayRotation(U8G2_R0);
  //u8g2.setFont(u8g_font_baby); //11 pixels high?
  u8g2.setFont(u8g_font_unifont); //11 pixels high?
  u8g2.setFontMode(1); //transparent font
  u8g2.clearBuffer();					// clear the internal memory
  u8g2.drawXBMP( car_x, car_y, car_width, car_height, car_bits);
  u8g2.drawXBMP( controller_x, controller_y, controller_width, controller_height, controller_bits);
  u8g2.drawXBMP( antenna_x, antenna_y, antenna_width, antenna_height, antenna_bits);
  u8g2.sendBuffer();         // transfer internal memory to the display
  
  //redraw(255,readBatt(),ARC);
}

/*
unsigned long fakeTimer=0;
int fakeThrottleInput = 0;
int fakeSpeedFactor = 1;
int fakeSteeringInput = 3;
*/
unsigned long oledTimer=0;
#define OLED_FRAMETIME_MS 500

void loop()
{
    float receiverBatteryVoltage=0;
    uint8_t receiverBatteryPercentage=0;

    uint32_t currentMicros = micros();
    crsfClass.update();

    RSSI_PERCENT=0;
    if(crsfClass.linkUP())
{
      const crsf_sensor_battery_t* batt = crsfClass.getBatt();
      receiverBatteryVoltage = (float)(batt->voltage)/10.0;
      receiverBatteryPercentage = batt->remaining;
      const crsfLinkStatistics_t* stat_ptr = crsfClass.getLinkStatistics();
      uint8_t RSSI = stat_ptr->downlink_RSSI;
      RSSI_PERCENT = map(RSSI,RSSI_WORST,RSSI_BEST,0,100);
      RSSI_PERCENT = constrain(RSSI_PERCENT, 0, 100);
}   




  int8_t speedFactor = checkPaddles(); //1, 2 or 3
  int8_t throttleInput = getThrottle(); //-3 to 3
  int8_t steeringInput = getSteering()*-1; //-3 to 3
  int8_t batteryPercent = readBatt();

if(millis()-oledTimer>OLED_FRAMETIME_MS){
  redraw(receiverBatteryPercentage, receiverBatteryVoltage, batteryPercent, RSSI_PERCENT);
  oledTimer=millis();
}
/*
  if(millis()-fakeTimer>250) {
    fakeThrottleInput++;
    fakeSteeringInput--;
    fakeTimer=millis();
    if (fakeThrottleInput>3) {
      fakeThrottleInput=-3;
      fakeSpeedFactor++;
      if(fakeSpeedFactor>3) fakeSpeedFactor=1;
    }
    if(fakeSteeringInput<-3) fakeSteeringInput=3;

    }  
  throttleInput=fakeThrottleInput;
  speedFactor=fakeSpeedFactor;
  steeringInput=fakeSteeringInput;
*/
  

                     //    1-3              -3-3
  int throttleDiff = speedFactor * 91 * throttleInput; // 91 = (CRSF_CHANNEL_VALUE_SPAN/2)/(3*3)
  
  int throttleVal = CRSF_CHANNEL_VALUE_MID + throttleDiff;

  //throttleVal = map(throttleInput,-3,3,CRSF_CHANNEL_VALUE_MIN,CRSF_CHANNEL_VALUE_MAX)

  int rudderDiff = 273*steeringInput; //273 = ((CRSF_CHANNEL_VALUE_SPAN/2)/3)
  int rudderVal = CRSF_CHANNEL_VALUE_MID + rudderDiff;

  int batteryChannel = map(batteryPercent,0,100,CRSF_CHANNEL_VALUE_MIN,CRSF_CHANNEL_VALUE_MAX);
  //TEST waves:
    //Throttle_value = (uint16_t)(((1.0+sin((float)millis()/1000.0))*0.5)*ADC_MAX);
    //Rudder_value = (uint16_t)(((1.0+cos((float)millis()/1000.0))*0.5)*ADC_MAX); 

    rcChannels[AILERON]   = constrain(batteryChannel,CRSF_CHANNEL_VALUE_MIN,CRSF_CHANNEL_VALUE_MAX); 
    rcChannels[THROTTLE]  = constrain(throttleVal,CRSF_CHANNEL_VALUE_MIN,CRSF_CHANNEL_VALUE_MAX);
    rcChannels[RUDDER]    = constrain(rudderVal,CRSF_CHANNEL_VALUE_MIN,CRSF_CHANNEL_VALUE_MAX);

    //Serial.print("speedFactor:"); Serial.print(speedFactor);
    //Serial.print("throttleInput:"); Serial.print(throttleInput);
    //Serial.print("throttleDiff:"); Serial.print(throttleDiff);
    //Serial.print("throttleVal:"); Serial.print(throttleVal);
    //Serial.print("THROTTLE:"); Serial.print(rcChannels[THROTTLE]);
    //Serial.print("steeringInput:"); Serial.print(steeringInput);
    //Serial.print(",STEERING:"); Serial.print(rcChannels[RUDDER]);
    //Serial.print(",RXbatt:"); Serial.print(receiverBatteryVoltage);
    //Serial.print(",RXbatt%:"); Serial.print(receiverBatteryPercentage);

    //Serial.print(",TXbatt:"); Serial.print((float)batteryPercent/100.0);
    //Serial.print(",TELEMETRY_RSSI%:"); Serial.print(RSSI_PERCENT);
    //Serial.print(",MIN:"); Serial.print(-3);
    //Serial.print(",MAX:"); Serial.print(9);
    //Serial.println();

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
                    crsfClass.crsfPrepareCmdPacket(crsfCmdPacket  , ELRS_PKT_RATE_COMMAND, currentPktRate);
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
  static uint8_t slowerPaddleDeBounceCounter=0;
  static uint8_t fasterPaddleDeBounceCounter=0;

  //Paddles need to have had a stable LOW reading for 20 consecutive calls to checkPaddles (about 20ms)
  if(!digitalRead(fasterPaddlePin)) fasterPaddleDeBounceCounter++;
  else fasterPaddleDeBounceCounter=0;
  if(!digitalRead(slowerPaddlePin)) slowerPaddleDeBounceCounter++;
  else slowerPaddleDeBounceCounter=0;

  if(fasterPaddleDeBounceCounter+slowerPaddleDeBounceCounter>0) delay(1); //

  if(fasterPaddleDeBounceCounter>20 && lastState_fasterPaddle) { //paddle has been pressed for more than 20mS
    lastState_fasterPaddle = LOW; //set last state to pressed
    speed++;
  }

  if(slowerPaddleDeBounceCounter>20 && lastState_slowerPaddle) { //paddle has been pressed for more than 20mS
    lastState_slowerPaddle = LOW; //set last state to pressed
    speed--;
  }

  if(speed>3) speed=3;
  else if(speed<1) speed=1;

  if(digitalRead(fasterPaddlePin) && !lastState_fasterPaddle) lastState_fasterPaddle = HIGH; //reset last state  
  if(digitalRead(slowerPaddlePin) && !lastState_slowerPaddle) lastState_slowerPaddle = HIGH; //reset last state

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

//Voltage divider: None.
#define V_BATTMAX 4.2
#define V_BATTMIN 3.0
#define V_ADCMAX 5.0
#define ADCMAX 1023.0
#define V_DIVIDER_MAX_OUT 4.2
#define DIVIDER_FACTOR 1.0
#define V_CALIBATED_OFFSET 0.09
#define N_MEASUREMENTS 32

uint8_t readBatt(){
  //Do a bunch of ADC measurements and convert them to average Vbatt, append Vbatt to report
  unsigned long ADCSum = 0;
  for (int i=0; i<N_MEASUREMENTS; i++) ADCSum+=analogRead(VbattPin); 
  unsigned int ADCavg = ADCSum/N_MEASUREMENTS;
  float vBatt = (float)ADCavg*V_ADCMAX/ADCMAX/DIVIDER_FACTOR+V_CALIBATED_OFFSET; 
          //Vbatt minimum = 3.0, VbattMaximum = 4.2
  //Serial.println(vBatt);
  return uint8_t(((vBatt - V_BATTMIN) * 100.0 / (V_BATTMAX - V_BATTMIN))); //calculate battery percentage
}

void redraw(uint8_t _rxBatt, float _rxVolts, uint8_t _txBatt, uint8_t _RSSI)
{
  static uint8_t lastRxPercent;
  static uint8_t lastTxPercent;
  static uint8_t highestRSSI;
  static uint8_t lastRSSI;
  

  //unsigned long in = millis();  
  /*
  Serial.print(millis());
  Serial.print(',');
  */
  //Trying to make a non-zero signal quality stick to the screen for longer:
  //shouldn't really do calculations in here, but the framerate is 1Hz, so it feels right...
  if(_RSSI<1){ //16 is the error state when there's no connection...
  if(_RSSI>highestRSSI) highestRSSI=_RSSI;
  else if(highestRSSI>0) highestRSSI--; //should reach 0 after a few seconds.

  _RSSI = highestRSSI; //sort of an injection of a new funtionality into old code.. sorry.
  }

/*
  Serial.print(lastTelemetryRXtime);
  Serial.print(',');
  Serial.print(millis()-lastTelemetryRXtime);
  Serial.print(',');
  Serial.print(rxBatt);
  Serial.print(',');
  Serial.print(txBatt);
  Serial.print(',');
  */
  
  
  //Serial.println(_ARC);
  if(_rxBatt != lastRxPercent || _txBatt != lastTxPercent || _RSSI != lastRSSI){

        u8g2.setDrawColor(0);
        u8g2.drawBox(car_width+4,1,100,12); //clear RX percentage bar
        u8g2.drawBox(controller_width+4,19,antenna_x-(controller_width+4),12); //clear TX percentage bar
        u8g2.drawBox(88,19,40,12); //clear ARC bar
        
    //Boxes:
    u8g2.setDrawColor(1);
    if(_rxBatt<101) u8g2.drawBox(24,1,_rxBatt,12); //RX
    u8g2.drawBox(controller_width+4,19,_txBatt/3,12);             //TX
    if(_RSSI>0) u8g2.drawBox(88,19,map(_RSSI,0,100,0,40),12);   //ARC
    
    //Text:
    u8g2.setDrawColor(2);

        //RX
        u8g2.setCursor(30,12);
        if(_rxBatt>100) u8g2.print("!");
        else {
          u8g2.print(_rxBatt);
          u8g2.print("% ");
          u8g2.print(String(_rxVolts/2.0,1));
          u8g2.print("V");
        }
        //TX
        u8g2.setCursor(30,30);
        u8g2.print(_txBatt);
        u8g2.print("%");

        //RSSI
        //u8g2.setCursor(104,30);
        u8g2.setCursor(92,30);
      if(_RSSI>0) {
        u8g2.print(_RSSI);
        u8g2.print("%");
      }
      else u8g2.print("!");


    lastRxPercent = _rxBatt;
    lastTxPercent = _txBatt;
    lastRSSI = _RSSI;
    u8g2.sendBuffer();         // transfer internal memory to the display
  }
  //Serial.println(millis()-in);

}
