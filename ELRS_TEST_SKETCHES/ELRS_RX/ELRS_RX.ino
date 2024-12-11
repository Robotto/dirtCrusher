/*
This example reads the voltage on pin PIN_SNS_VIN and sends it to your radio in a telemetry packet
This code assumes you are using a voltage divider with a "high side" resistance of 15K and "low side" resistance of 2.2K
*/

#include <AlfredoCRSF.h>

#define PIN_RX 0
#define PIN_TX 1

#define PIN_SNS_VIN A0

#define RESISTOR1 15000.0
#define RESISTOR2 2200.0
#define ADC_RES 8192.0
#define ADC_VLT 3.3

// Set up a new Serial object
//HardwareSerial crsfSerial(Serial1);
AlfredoCRSF crsf;

void setup()
{
  Serial.begin(250000);
   Serial.println("COM Serial initialized");
  
  Serial1.begin(250000);
  if (!Serial1) while (1) Serial.println("Invalid crsfSerial configuration");

  crsf.begin(Serial1);
}

float cap = 0;


float RSSI_WORST=108.0; //dBm (negative)
float RSSI_BEST=50.0;  //dBm (negative)
float RSSI_PERCENT=0;
int throttle = 0;
int rudder = 0;

void loop()
{ 
  // Must call crsf.update() in loop() to process data
  crsf.update();

  const crsfLinkStatistics_t* stat_ptr = crsf.getLinkStatistics();
  uint8_t RSSI_1 = stat_ptr->uplink_RSSI_1;
  uint8_t RSSI_2 = stat_ptr->uplink_RSSI_2;
  uint8_t RSSI = min(RSSI_1,RSSI_2);
  RSSI_PERCENT = map(RSSI,RSSI_WORST,RSSI_BEST,0,100);
  RSSI_PERCENT = constrain(RSSI_PERCENT, 0, 100);
  //uint8_t LQ = stat_ptr->uplink_Link_quality;

if(crsf.isLinkUp()){
  uint16_t rxThrottle = crsf.getChannel(3);
  throttle = 0;
  if (rxThrottle > 1600) throttle=1;
  if (rxThrottle > 1900) throttle=2;
  if (rxThrottle < 1400) throttle=-1;
  if (rxThrottle < 1100) throttle=-2;
  
  uint16_t rxRudder = crsf.getChannel(4);
  rudder = 0;
  if (rxRudder > 1600) rudder=1;
  if (rxRudder > 1900) rudder=2;
  if (rxRudder < 1400) rudder=-1;
  if (rxRudder < 1100) rudder=-2;


  int snsVin = analogRead(PIN_SNS_VIN);
  float batteryVoltage = ((float)snsVin * ADC_VLT / ADC_RES) * ((RESISTOR1 + RESISTOR2) / RESISTOR2);
  sendRxBattery(batteryVoltage, 1.2, cap += 10, 50);
} //https://github.com/crsf-wg/crsf/wiki/CRSF_FRAMETYPE_LINK_STATISTICS
else{
   throttle=0;
   rudder=0;
}

  Serial.print("Throttle:");
  Serial.print(throttle);
  Serial.print(",Rudder:");
  Serial.print(rudder);
  Serial.print(",RSSI%:");
  Serial.print((float)RSSI_PERCENT/100.0);

  Serial.print(",MIN:");
  Serial.print(-2);
  Serial.print(",MAX:");
  Serial.println(2); //DUMMY VALUE TO STOP SERIAL PLOTTER FROM AUTOSCALING...


  delay(10);
}

static void sendRxBattery(float voltage, float current, float capacity, float remaining)
{
  crsf_sensor_battery_t crsfBatt = { 0 };

  // Values are MSB first (BigEndian)
  crsfBatt.voltage = htobe16((uint16_t)(voltage * 10.0));   //Volts
  crsfBatt.current = htobe16((uint16_t)(current * 10.0));   //Amps
  crsfBatt.capacity = htobe16((uint16_t)(capacity)) << 8;   //mAh (with this implemetation max capacity is 65535mAh)
  crsfBatt.remaining = (uint8_t)(remaining);                //percent
  crsf.queuePacket(CRSF_SYNC_BYTE, CRSF_FRAMETYPE_BATTERY_SENSOR, &crsfBatt, sizeof(crsfBatt));
}