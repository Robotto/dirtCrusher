#include "src/drv8871/drv8871.h"
#include "src/AlfredoCRSF/AlfredoCRSF.h"
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

const int VbattPin = A2;
//Constants:
const int failsafeTimeout = 250;  //ms


AlfredoCRSF crsf;

//Variables:
//uint8_t rx=0;
int rssiPWM = 0;
float RSSI_WORST = 108.0;  //dBm (negative)
int rssi = RSSI_WORST;
float RSSI_BEST = 50.0;  //dBm (negative)
float RSSI_PERCENT = 0;
unsigned long lastTelemetryTXtime = 0;

unsigned long nextFailsafeTimeout = 0;
int previousState = 0;  //holds steering feedback position from last read.

const int throttlePWM_MAX = 62;
const int throttlePWM_MIN = 32;
const int throttlePWM_SPAN = (throttlePWM_MAX - throttlePWM_MIN);
const int throttlePWM_MID = throttlePWM_MIN + throttlePWM_SPAN / 2;

const int THROTTLE_DEADZONE = 10;
const int RUDDER_DEADZONE = 250;

int throttlePWM = throttlePWM_MID;

int steering = 0;


//Objects:
DRV8871 steeringDriver(steeringDriverPin1, steeringDriverPin2);

void noTurn() {
  steeringDriver.brake();
}

void turnLeft() {
  steeringDriver.reverse();
}

void turnRight() {
  steeringDriver.forward();
}



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
  pinMode(steeringFeedbackPinA, INPUT_PULLUP);
  pinMode(steeringFeedbackPinB, INPUT_PULLUP);
  pinMode(steeringFeedbackPinC, INPUT_PULLUP);
  pinMode(PWMmotorPin, OUTPUT);
  pinMode(PWMrssiPin, OUTPUT);

  Serial.println("Good to go!");
}

int txBatt;
int txBatt_previous;

void loop() {
  int16_t rxThrottle = CRSF_CHANNEL_VALUE_MID;
  int16_t rxRudder = CRSF_CHANNEL_VALUE_MID;
  // Must call crsf.update() in loop() to process data
  crsf.update();



  if (crsf.isLinkUp()) {  //TODO: Check if failsafe is a thing?!

    const crsfLinkStatistics_t* stat_ptr = crsf.getLinkStatistics();
    uint8_t RSSI_1 = stat_ptr->uplink_RSSI_1;
    uint8_t RSSI_2 = stat_ptr->uplink_RSSI_2;
    uint8_t RSSI = min(RSSI_1, RSSI_2);
    RSSI_PERCENT = map(RSSI, RSSI_WORST, RSSI_BEST, 0, 100);
    RSSI_PERCENT = constrain(RSSI_PERCENT, 0, 100);

    nextFailsafeTimeout = millis() + failsafeTimeout;




    //uint8_t LQ = stat_ptr->uplink_Link_quality;


    rxThrottle = crsf.getChannel(3);  //INDEXED BY 1 YOU PIECE OF SHIIIIT!



    rxRudder = crsf.getChannel(4);

    txBatt = map(crsf.getChannel(1),CRSF_CHANNEL_VALUE_MIN,CRSF_CHANNEL_VALUE_MAX,0,100);
    
    if (abs(txBatt-txBatt_previous)<4) txBatt=txBatt_previous; //if battery percentage has moved less than 5%, we stay on previous value
    
    txBatt_previous=txBatt;






    //int snsVin = analogRead(PIN_SNS_VIN);
    // float batteryVoltage = ((float)snsVin * ADC_VLT / ADC_RES) * ((RESISTOR1 + RESISTOR2) / RESISTOR2);
    if (millis() - lastTelemetryTXtime > 250) {
      uint16_t batteryVoltage = (uint16_t)(((1.0 + sin((float)millis() / 1000.0)) * 0.5) * 10.0);
      sendRxBattery(batteryVoltage, 1.2, 0, 50);
      lastTelemetryTXtime = millis();
    }

  }  //https://github.com/crsf-wg/crsf/wiki/CRSF_FRAMETYPE_LINK_STATISTICS
  else {
    throttlePWM = throttlePWM_MID;
    steering = 0;
    RSSI_PERCENT = 0.0;
    txBatt=0;
  }

  //HANDLE RSSI:
  rssiPWM = map(RSSI_PERCENT, 0, 100, 0, 255);
  analogWrite(PWMrssiPin, rssiPWM);

  //HANDLE THROTTLE:
  if (abs(rxThrottle - CRSF_CHANNEL_VALUE_MID) < THROTTLE_DEADZONE) rxThrottle = CRSF_CHANNEL_VALUE_MID;
  throttlePWM = map(rxThrottle, CRSF_CHANNEL_VALUE_MIN, CRSF_CHANNEL_VALUE_MAX, throttlePWM_MIN, throttlePWM_MAX);
  analogWrite(PWMmotorPin, throttlePWM);  //TODO: Determine if deadzone is large enough


  //HANDLE STEERING:
  if (abs(rxRudder - CRSF_CHANNEL_VALUE_MID) < RUDDER_DEADZONE) rxRudder = CRSF_CHANNEL_VALUE_MID;
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
  previousState = readSteeringFeedback();
  int steeringDelta = steering - previousState;
  //steeringDelta == 0;                ///////////////////////////////////TODO: THIS IS FOR TESTING!!
  if (steeringDelta == 0) noTurn();  //need to go nowhere
  else if (steeringDelta < 0) turnLeft();
  else if (steeringDelta > 0) turnRight();
  

  
  //Serial.print("rxThrottle:"); Serial.print(rxThrottle);
  //Serial.print("Throttle(PWM):"); Serial.print((((float)throttlePWM)-32.0)/30.0);

  //Serial.print("rxRudder:"); Serial.print(rxRudder);
  
  //Serial.print(",Steering:"); Serial.print((float)(steering+3.0)/6.0);
  
  //Serial.print(",RSSI:"); Serial.print(((float)RSSI_PERCENT/100));

  //Serial.print(",TXBATT%:"); Serial.print(((float)txBatt)/100);


  //Serial.print(",MIN:");
  //Serial.print(0);
  //Serial.print(",MAX:");
  //Serial.print(1);  //DUMMY VALUE TO STOP SERIAL PLOTTER FROM AUTOSCALING...

  //Serial.println();
  delay(10);
}


int readSteeringFeedback() {  //negative is turning left!
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
  if (aState & bState & cState) return previousState;  //non-discrete in-between-state with all pins high - Keep moving the steering.
  //Now we know the feedback is in a discrete state:
  if (bState) return 2 - cState + aState;  //Steering feedback is positive (1,2,3)
  else if (cState & !aState) return 0;     // A=0, B=0; C=1
  else return -3 + aState + cState;        //-3,-2,-1
}

static void sendRxBattery(float voltage, float current, float capacity, float remaining) {
  crsf_sensor_battery_t crsfBatt = { 0 };

  // Values are MSB first (BigEndian)
  crsfBatt.voltage = htobe16((uint16_t)(voltage)*10.0);    //Volts
  crsfBatt.current = htobe16((uint16_t)(current * 10.0));  //Amps
  crsfBatt.capacity = htobe16((uint16_t)(capacity)) << 8;  //mAh (with this implemetation max capacity is 65535mAh)
  crsfBatt.remaining = (uint8_t)(remaining);               //percent
  crsf.queuePacket(CRSF_ADDRESS_FLIGHT_CONTROLLER, CRSF_FRAMETYPE_BATTERY_SENSOR, &crsfBatt, sizeof(crsfBatt));
}

//Voltage divider: None.
#define V_BATTMAX 8.4
#define V_BATTMIN 3.5
#define V_ADCMAX 5.0
#define ADCMAX 1023.0
#define V_DIVIDER_MAX_OUT 4.2
#define DIVIDER_FACTOR 2.0
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
  //return (uint8_t)50;
  return uint8_t(((vBatt - V_BATTMIN) * 100.0 / (V_BATTMAX - V_BATTMIN))); //calculate battery percentage
}
