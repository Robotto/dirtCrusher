//#include "src/drv8871/drv8871.h"
#include "src/AlfredoCRSF/AlfredoCRSF.h"
#define CRSF_DIGITAL_CHANNEL_MIN 172
#define CRSF_DIGITAL_CHANNEL_MAX 1811

//Pins:
const int LEFT_PWMmotorPin = 9;
const int RIGHT_PWMmotorPin = 10;
const int VbattPin = A2;

//Constants:
const int failsafeTimeout = 250;  //ms
const int LEFT_throttlePWM_MAX = 62;
const int LEFT_throttlePWM_MIN = 32;
const int LEFT_throttlePWM_SPAN = (LEFT_throttlePWM_MAX - LEFT_throttlePWM_MIN);
const int LEFT_throttlePWM_MID = LEFT_throttlePWM_MIN + LEFT_throttlePWM_SPAN / 2;

const int RIGHT_throttlePWM_MAX = 62;
const int RIGHT_throttlePWM_MIN = 32;
const int RIGHT_throttlePWM_SPAN = (RIGHT_throttlePWM_MAX - RIGHT_throttlePWM_MIN);
const int RIGHT_throttlePWM_MID = RIGHT_throttlePWM_MIN + RIGHT_throttlePWM_SPAN / 2;

const int THROTTLE_DEADZONE = 10;
const int RUDDER_DEADZONE = 250;

//Variables:
//float RSSI_WORST = 108.0;  //dBm (negative)
//int rssi = RSSI_WORST;
//float RSSI_BEST = 50.0;  //dBm (negative)
//float RSSI_PERCENT = 0;

unsigned long lastTelemetryTXtime = 0;

unsigned long nextFailsafeTimeout = 0;
int previousState = 0;  //holds steering feedback position from last read.

int LEFT_throttlePWM = LEFT_throttlePWM_MID;
int RIGHT_throttlePWM = RIGHT_throttlePWM_MID;

int steering = 0;
int gear = 1; //1,2,3



//Objects:
AlfredoCRSF crsf;

void setup() {
  Serial.begin(250000);
  Serial.println("COM Serial initialized");

  Serial1.begin(250000);
  if (!Serial1)
    while (1) Serial.println("Invalid crsfSerial configuration");

  crsf.begin(Serial1);

  /* PRESCALE DIVIDER FOR PWM FRQ ON PINS 9 and 10:
  16'000'000 / 256 / 2 = 31250Hz 
  0x01 1 31250
  0x02 8 3906.25
  0x03 64 488.28125
  0x04 256 122.0703125
  0x05 1024 30.517578125
  */
  TCCR1B = TCCR1B & 0b11111000 | 0x04;  // <- Works.
  //TCCR1B = TCCR1B & 0b11111000 | 0x05;
  
  pinMode(LEFT_PWMmotorPin, OUTPUT);
  pinMode(RIGHT_PWMmotorPin, OUTPUT);

  Serial.println("Good to go!");
}

//int txBatt;
//int txBatt_previous;
float vBatt;

void loop() {
  int16_t rxThrottle = CRSF_CHANNEL_VALUE_MID;
  int16_t rxRudder = CRSF_CHANNEL_VALUE_MID;
  int16_t rxGear = CRSF_DIGITAL_CHANNEL_MIN; //lowest gear

  // Must call crsf.update() in loop() to process data
  crsf.update();

  if (crsf.isLinkUp()) {  //TODO: Check if failsafe is a thing?!
//https://github.com/crsf-wg/crsf/wiki/CRSF_FRAMETYPE_LINK_STATISTICS
  /*
    const crsfLinkStatistics_t* stat_ptr = crsf.getLinkStatistics();
    uint8_t RSSI_1 = stat_ptr->uplink_RSSI_1;
    uint8_t RSSI_2 = stat_ptr->uplink_RSSI_2;
    uint8_t RSSI = min(RSSI_1, RSSI_2);
    RSSI_PERCENT = map(RSSI, RSSI_WORST, RSSI_BEST, 0, 100);
    RSSI_PERCENT = constrain(RSSI_PERCENT, 0, 100);
  */
    nextFailsafeTimeout = millis() + failsafeTimeout;

    rxThrottle = crsf.getChannel(3);  //INDEXED BY 1 YOU PIECE OF SHIIIIT!
    rxRudder = crsf.getChannel(4);
    rxGear = crsf.getChannel(2); //which gear are we in? 1,2,3
    //txBatt = map(crsf.getChannel(1),CRSF_CHANNEL_VALUE_MIN,CRSF_CHANNEL_VALUE_MAX,0,100);
    //if (abs(txBatt-txBatt_previous)<4) txBatt=txBatt_previous; //if battery percentage has moved less than 5%, we stay on previous value
    //txBatt_previous=txBatt;


  
    if (millis() - lastTelemetryTXtime > 250) {
      //uint16_t batteryVoltage = (uint16_t)(((1.0 + sin((float)millis() / 1000.0)) * 0.5) * 10.0);
      uint8_t batteryPercentage = readBatt();
      
      sendRxBattery(vBatt, 0, 0, (float)batteryPercentage);
      lastTelemetryTXtime = millis();
    }

  }  
  else {
    LEFT_throttlePWM = LEFT_throttlePWM_MID;
    RIGHT_throttlePWM = RIGHT_throttlePWM_MID;
    rxGear = CRSF_DIGITAL_CHANNEL_MIN;
    steering = 0;
    //RSSI_PERCENT = 0.0;
    //txBatt=0;
    static unsigned long notConnectedPrintTime = 0;
    if(millis() > notConnectedPrintTime+3000) { Serial.println("CRSF link is down@" + String(millis())); notConnectedPrintTime=millis();}
  }


  //handle deadzone:
  if (abs(rxThrottle - CRSF_CHANNEL_VALUE_MID) < THROTTLE_DEADZONE) rxThrottle = CRSF_CHANNEL_VALUE_MID;
  if (abs(rxRudder - CRSF_CHANNEL_VALUE_MID) < RUDDER_DEADZONE) rxRudder = CRSF_CHANNEL_VALUE_MID;
  if (abs(rxGear - CRSF_CHANNEL_VALUE_MID) < RUDDER_DEADZONE) rxGear = CRSF_CHANNEL_VALUE_MID;
  
  //HANDLE Gear:
  gear=1;
  if ( rxGear == CRSF_CHANNEL_VALUE_MID) gear = 2;
  if ( rxGear > CRSF_CHANNEL_VALUE_MID) gear = 3;


  //HANDLE THROTTLE:
  
  //Calculate throttle before adding/subtracting steering values:
  LEFT_throttlePWM = map(rxThrottle, CRSF_CHANNEL_VALUE_MIN, CRSF_CHANNEL_VALUE_MAX, LEFT_throttlePWM_MIN, LEFT_throttlePWM_MAX);
  RIGHT_throttlePWM = map(rxThrottle, CRSF_CHANNEL_VALUE_MIN, CRSF_CHANNEL_VALUE_MAX, RIGHT_throttlePWM_MIN, RIGHT_throttlePWM_MAX);
  

  //HANDLE STEERING:
  
  //steering = map(rxRudder, CRSF_CHANNEL_VALUE_MIN+2, CRSF_CHANNEL_VALUE_MAX-2, -3, 3); //for some reason the mapping was a bit off...
  switch (rxRudder){
    case 174:
      steering = -3;
      break;
    case 446:
      steering = -2;
      break;
    case 718:
      steering = -1;
      break;
    case  992: //CRSF_CHANNEL_VALUE_MID
      steering = 0;
      break;
    case 1265:
      steering = 1;
      break;
    case 1539:
      steering = 2;
      break;
    case 1811:
      steering = 3;
      break;  
    default:
      steering=0;
      break;
  }
  
  //Steering authority should be inversely proportional to the speed:
                                  // 0, 1,  2,  3
  float steeringAuthorityPerGear[4]={0,1.9,1.76,1.5};

  float steeringAuthority=steeringAuthorityPerGear[gear];

  //Calculate throttle percentage to determine how much to steer:
  float throttleFullness = (map(rxThrottle,CRSF_CHANNEL_VALUE_MIN,CRSF_CHANNEL_VALUE_MAX,0,2000)-1000)/1000.0;

  if(steering!=0) {
    if(throttleFullness>0){

      if(steering<0) LEFT_throttlePWM += (int)((steeringAuthority-throttleFullness)*(float)steering);
      else if (steering>0) RIGHT_throttlePWM -= (int)((steeringAuthority-throttleFullness)*(float)steering);
    
    }
    else if(throttleFullness<0){
    
      if(steering<0) RIGHT_throttlePWM -= (int)((steeringAuthority-throttleFullness)*(float)steering);
      else if (steering>0) LEFT_throttlePWM += (int)((steeringAuthority-throttleFullness)*(float)steering);
    
    }
    else{
      LEFT_throttlePWM = LEFT_throttlePWM_MID + (int)((float)steering/(float)gear+(float)steering*(float)gear*steeringAuthority+0.3/(float)steering);
      RIGHT_throttlePWM = RIGHT_throttlePWM_MID - (int)((float)steering/(float)gear+(float)steering*(float)gear*steeringAuthority+0.3/(float)steering);
    }
  }
  
  analogWrite(LEFT_PWMmotorPin, LEFT_throttlePWM);  
  analogWrite(RIGHT_PWMmotorPin, RIGHT_throttlePWM);  
//  analogWrite(LEFT_PWMmotorPin, LEFT_throttlePWM_MID); 
//  analogWrite(RIGHT_PWMmotorPin, RIGHT_throttlePWM_MID);  


  /*
  Serial.print("Gear:"); Serial.print(gear);
  Serial.print("\trxGear:"); Serial.print(rxGear);
  Serial.print("\trxThrottle:"); Serial.print(rxThrottle);
  Serial.print("\trxRudder:"); Serial.print(rxRudder);
  
 
  Serial.print("\tSteering:"); Serial.print((steering));
  //Serial.print("\tThrottle:"); Serial.print((throttleFullness));

  Serial.print("\tL_dPWM:"); Serial.print(((float)LEFT_throttlePWM-LEFT_throttlePWM_MID));
  Serial.print("\tR_dPWM:"); Serial.print(((float)RIGHT_throttlePWM-RIGHT_throttlePWM_MID));
  
  //Serial.print("\tRSSI:"); Serial.print(((float)RSSI_PERCENT));

  //Serial.print("\tTXBATT%:"); Serial.print(((float)txBatt));
  
  //Serial.print("\tVbatt:"); Serial.print(vBatt);
  //Serial.print("\tBATT%:"); Serial.print(readBatt());

  //DUMMY VALUES TO STOP SERIAL PLOTTER FROM AUTOSCALING...
  //Serial.print("\tMIN:"); Serial.print(0);
  //Serial.print("\tMAX:"); Serial.print(100);  

  Serial.println();
  delay(1);
  */

}

static void sendRxBattery(float voltage, float current, float capacity, float remaining) {
  crsf_sensor_battery_t crsfBatt = { 0 };

  // Values are MSB first (BigEndian)
  crsfBatt.voltage = htobe16((uint16_t)(voltage*10.0));    //Volts
  crsfBatt.current = htobe16((uint16_t)(current * 10.0));  //Amps
  crsfBatt.capacity = htobe16((uint16_t)(capacity)) << 8;  //mAh (with this implemetation max capacity is 65535mAh)
  crsfBatt.remaining = (uint8_t)(remaining);               //percent
  crsf.queuePacket(CRSF_ADDRESS_FLIGHT_CONTROLLER, CRSF_FRAMETYPE_BATTERY_SENSOR, &crsfBatt, sizeof(crsfBatt));
}

//Voltage divider: Vbatt - 10K - 10K - A2 - 10K - GND.
#define V_BATTMAX 8.4f
#define V_BATTMIN 6.0f
#define V_ADCMAX 5.0f
#define ADCMAX 1023.0f
#define V_DIVIDER_MAX_OUT 2.8f //8.4/3
#define DIVIDER_FACTOR 0.33f
#define V_CALIBATED_OFFSET -0.07f
#define N_MEASUREMENTS 16


uint8_t readBatt(){
  //Do a bunch of ADC measurements and convert them to average Vbatt, append Vbatt to report
  unsigned long ADCSum = 0;
  for (int i=0; i<N_MEASUREMENTS; i++) ADCSum+=analogRead(VbattPin); 
  unsigned int ADCavg = ADCSum/N_MEASUREMENTS;
  //Serial.println(ADCavg);
  vBatt = (float)ADCavg*V_ADCMAX/ADCMAX/DIVIDER_FACTOR+V_CALIBATED_OFFSET; 
          //Vbatt minimum = 3.0, VbattMaximum = 4.2
  //Serial.println(vBatt);
  //return (uint8_t)50;
  return uint8_t(((vBatt - V_BATTMIN) * 100.0 / (V_BATTMAX - V_BATTMIN))); //calculate battery percentage
}
