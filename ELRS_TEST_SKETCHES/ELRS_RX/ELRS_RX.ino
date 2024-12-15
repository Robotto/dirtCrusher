#include "drv8871.h"

#include <AlfredoCRSF.h>
#define CRSF_DIGITAL_CHANNEL_MIN 172
#define CRSF_DIGITAL_CHANNEL_MAX 1811
//Pins:
const int steeringFeedbackPinA = 4;
const int steeringFeedbackPinB = 5;
const int steeringFeedbackPinC = 6;
const int steeringDriverPin1 = 7;
const int steeringDriverPin2 = 8;
const int PWMmotorPin = 9;
const int PWMrssiPin = 10;

//Constants:
const int failsafeTimeout = 250; //ms
const unsigned int STOP_PWM_VAL = 127;

AlfredoCRSF crsf;

//Variables:
//uint8_t rx=0;
int rssiPWM = 0;
float RSSI_WORST=108.0; //dBm (negative)
int rssi=RSSI_WORST;
float RSSI_BEST=50.0;  //dBm (negative)
float RSSI_PERCENT=0;

unsigned long nextFailsafeTimeout = 0;
int previousState = 0; //holds steering feedback position from last read.

int throttle = 127;
int steering = 0;


//Objects:
DRV8871 steeringDriver(steeringDriverPin1,steeringDriverPin2);

void noTurn(){
  steeringDriver.brake(); 
}

void turnLeft(){
  steeringDriver.reverse();
}

void turnRight(){
  steeringDriver.forward();
}



void setup()
{
  Serial.begin(250000);
   Serial.println("COM Serial initialized");
  
  Serial1.begin(250000);
  if (!Serial1) while (1) Serial.println("Invalid crsfSerial configuration");

  crsf.begin(Serial1);

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




void loop()
{ 
  // Must call crsf.update() in loop() to process data
  crsf.update();

  

if(crsf.isLinkUp()){ //TODO: Check if failsafe is a thing?!

  const crsfLinkStatistics_t* stat_ptr = crsf.getLinkStatistics();
  uint8_t RSSI_1 = stat_ptr->uplink_RSSI_1;
  uint8_t RSSI_2 = stat_ptr->uplink_RSSI_2;
  uint8_t RSSI = min(RSSI_1,RSSI_2);
  RSSI_PERCENT = map(RSSI,RSSI_WORST,RSSI_BEST,0,100);
  RSSI_PERCENT = constrain(RSSI_PERCENT, 0, 100);
  
  nextFailsafeTimeout = millis() + failsafeTimeout;




  //uint8_t LQ = stat_ptr->uplink_Link_quality;


  uint16_t rxThrottle = crsf.getChannel(3);
  throttle = map(rxThrottle,CRSF_DIGITAL_CHANNEL_MIN,CRSF_DIGITAL_CHANNEL_MAX,0,255);


  
  uint16_t rxRudder = crsf.getChannel(4);
  steering = map(rxRudder,CRSF_DIGITAL_CHANNEL_MIN,CRSF_DIGITAL_CHANNEL_MAX, 0, 6)-3; 




/*
  int snsVin = analogRead(PIN_SNS_VIN);
 // float batteryVoltage = ((float)snsVin * ADC_VLT / ADC_RES) * ((RESISTOR1 + RESISTOR2) / RESISTOR2);
  if(millis()-lastTXtime>250){
  batteryVoltage=(uint16_t)(((1.0+sin((float)millis()/1000.0))*0.5)*10.0);
  sendRxBattery(batteryVoltage, 1.2, cap += 10, 50);
  lastTXtime=millis();
  }
*/
} //https://github.com/crsf-wg/crsf/wiki/CRSF_FRAMETYPE_LINK_STATISTICS
else{
   throttle=STOP_PWM_VAL;
   steering=0;
   RSSI_PERCENT=0.0;
}

  //HANDLE RSSI:
  rssiPWM = map(RSSI_PERCENT,0,100,0,255);
  analogWrite(PWMrssiPin,rssiPWM);

  //HANDLE THROTTLE:
  int mapVal = map(throttle,0,255,32,62);
  analogWrite(PWMmotorPin, mapVal); //TODO: Determine if deadzone is large enough

  //HANDLE STEERING:
  previousState = readSteeringFeedback();
  int steeringDelta = steering-previousState; 
  if(steeringDelta==0) noTurn(); //need to go nowhere  
  else if(steeringDelta<0) turnLeft();
  else if(steeringDelta>0) turnRight();

  Serial.print("Throttle:");
  Serial.print(throttle);
  Serial.print(",Steering:");
  Serial.print(steering+3);
  Serial.print(",RSSI%:");
  Serial.print(((float)RSSI_PERCENT));
  

  Serial.print(",MIN:");
  Serial.print(0);
  Serial.print(",MAX:");
  Serial.println(100); //DUMMY VALUE TO STOP SERIAL PLOTTER FROM AUTOSCALING...


  delay(10);
}


int readSteeringFeedback(){ //negative is turning left!
  int aState = digitalRead(steeringFeedbackPinA);
  int bState = digitalRead(steeringFeedbackPinB);
  int cState = digitalRead(steeringFeedbackPinC);
/*
  Serial.print(aState);
  Serial.print(" "); 
  Serial.print(bState);
  Serial.print(" "); 
  Serial.println(cState);
*/
  if( aState & bState & cState ) return previousState; //non-discrete in-between-state with all pins high - Keep moving the steering.
  //Now we know the feedback is in a discrete state:
  if( bState ) return 2-cState+aState; //Steering feedback is positive (1,2,3)
  else if ( cState & !aState ) return 0; // A=0, B=0; C=1
  else return -3+aState+cState; //-3,-2,-1
}

static void sendRxBattery(float voltage, float current, float capacity, float remaining)
{
  crsf_sensor_battery_t crsfBatt = { 0 };

  // Values are MSB first (BigEndian)
  crsfBatt.voltage = htobe16((uint16_t)(voltage) * 10.0);   //Volts
  crsfBatt.current = htobe16((uint16_t)(current * 10.0));   //Amps
  crsfBatt.capacity = htobe16((uint16_t)(capacity)) << 8;   //mAh (with this implemetation max capacity is 65535mAh)
  crsfBatt.remaining = (uint8_t)(remaining);                //percent
  crsf.queuePacket(CRSF_ADDRESS_FLIGHT_CONTROLLER, CRSF_FRAMETYPE_BATTERY_SENSOR, &crsfBatt, sizeof(crsfBatt));
}