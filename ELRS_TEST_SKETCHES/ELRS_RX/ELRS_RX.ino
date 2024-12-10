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
  Serial.begin(115200);
   Serial.println("COM Serial initialized");
  
  Serial1.begin(420000);
  if (!Serial1) while (1) Serial.println("Invalid crsfSerial configuration");

  crsf.begin(Serial1);
}

float cap = 0;
void loop()
{
  
  // Must call crsf.update() in loop() to process data
  crsf.update();
if(crsf.isLinkUp()){
  uint8_t RSSI = crsf.getLinkStatistics()->downlink_RSSI;
  uint8_t LQ = crsf.getLinkStatistics()->downlink_Link_quality;
  int8_t SNR = crsf.getLinkStatistics()->downlink_SNR;
  Serial.print("RSSI="); Serial.print(String(RSSI)); Serial.print(",LQ="); Serial.print(String(LQ)); Serial.print(",SNR="); Serial.println(String(SNR));
  }
  int snsVin = analogRead(PIN_SNS_VIN);
  float batteryVoltage = ((float)snsVin * ADC_VLT / ADC_RES) * ((RESISTOR1 + RESISTOR2) / RESISTOR2);
  sendRxBattery(batteryVoltage, 1.2, cap += 10, 50);

//for(int i=0;i<17;i++)  Serial.println(crsf.getChannel(i));
//Serial.println();
Serial.println(crsf.getChannel(3));
//  Serial.print("RX buffer: "); Serial.println(Serial1.available());


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