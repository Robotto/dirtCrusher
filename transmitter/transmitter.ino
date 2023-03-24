#include <SPI.h>
#include "RF24.h"
const unsigned int NRF_CE_PIN = 10;
const unsigned int NRF_CSN_PIN = A1;
RF24 radio(NRF_CE_PIN, NRF_CSN_PIN); // using pin 8 for the CE pin, and pin 8 for the CSN pin
uint8_t address[][3] = {"TX", "RX"};
// uniquely identify which address this radio will use to transmit
bool radioNumber = 0; // 0 uses address[0] to transmit, 1 uses address[1] to transmit

#include <U8g2lib.h>
U8G2_SSD1306_128X32_UNIVISION_F_HW_I2C u8g2(U8G2_R0); 

//https://forum.arduino.cc/t/u8glib-and-bitmap-creation-display/148125/2

//CAR ICON:
#define car_width 20
#define car_height 15
#define car_x 0
#define car_y 0
static const unsigned char car_bits[] U8X8_PROGMEM = {
   0x3c, 0xcf, 0x03, 0xe0, 0x7f, 0x00, 0xf0, 0xff, 0x00, 0x38, 0xc0, 0x00,
   0x18, 0xc0, 0x01, 0x1f, 0x80, 0x0f, 0xfe, 0xff, 0x07, 0xfe, 0xff, 0x07,
   0xe6, 0x7f, 0x06, 0xc2, 0x3f, 0x04, 0xe6, 0x7f, 0x06, 0xfe, 0xff, 0x07,
   0xfc, 0xff, 0x03, 0x1c, 0x80, 0x03, 0x1c, 0x80, 0x03 };

//CONTROLLER ICON:
#define controller_width 20
#define controller_height 14
#define controller_x 0
#define controller_y car_height+3
static const unsigned char controller_bits[] U8X8_PROGMEM = {
   0x00, 0x02, 0x00, 0x00, 0x02, 0x00, 0x00, 0x02, 0x00, 0xf0, 0xff, 0x00,
   0x0c, 0x00, 0x03, 0x02, 0x00, 0x04, 0x01, 0x00, 0x08, 0x11, 0x40, 0x08,
   0x39, 0xe0, 0x08, 0x11, 0x40, 0x08, 0x01, 0x00, 0x08, 0x02, 0x00, 0x04,
   0xc4, 0x1f, 0x02, 0x38, 0xe0, 0x01 };

#define antenna_width 20
#define antenna_height 15
#define antenna_x 64
#define antenna_y car_height+3
static const unsigned char antenna_bits[] U8X8_PROGMEM = {
  0x08, 0x00, 0x01, 0x0C, 0x00, 0x03, 0x26, 0x40, 0x06, 0x36, 0xC0, 0x06, 
  0x33, 0xC6, 0x0C, 0x13, 0x8F, 0x0C, 0x93, 0x9F, 0x0C, 0x13, 0x8F, 0x0C, 
  0x37, 0xC6, 0x0E, 0x26, 0x46, 0x06, 0x0E, 0x06, 0x07, 0x0C, 0x06, 0x03, 
  0x00, 0x06, 0x00, 0x00, 0x06, 0x00, 0x00, 0x0F, 0x00};


const int ADCpin = A0;
const int xPin = 8; //ORANGE 
const int yPin = 5; //GREEN
const int throttle_aPin = A3; //yellow
const int throttle_bPin = A2; //WHITE
const int steering_aPin = 7; //yellow
const int steering_bPin = 4; //white
const int fasterPaddlePin = 6; //blue
const int slowerPaddlePin = 9; //pink 

#define TXPERIOD 20UL
#define FRAMERATE 1000UL

unsigned long lastTXtime = 0;
unsigned long lastRedraw = 0;
uint8_t lastPayload;
uint8_t ARC=16; //automatic retransmission count.. to be used as a rough RSSI/link quality estimate.



// stick states: 3 is center (neutral)
// 0 1 2 3 4 5 6
uint8_t speed = 1;
//uint8_t payload = 0b01011011;

                /*
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
//  Serial1.begin(9600);

  u8g2.begin();
  u8g2.setDisplayRotation(U8G2_R2);
  //u8g2.setFont(u8g_font_baby); //11 pixels high?
  u8g2.setFont(u8g_font_unifont); //11 pixels high?
  u8g2.setFontMode(1); //transparent font
  
  if (!radio.begin()) {
        Serial.println(F("radio hardware is not responding!!"));
        u8g2.print("NRF failed to init.");
        u8g2.sendBuffer();         // transfer internal memory to the display
        while (1) {} // hold in infinite loop
    }
  
  nrf24Setup();
  
  u8g2.clearBuffer();					// clear the internal memory
  u8g2.drawXBMP( car_x, car_y, car_width, car_height, car_bits);
  u8g2.drawXBMP( controller_x, controller_y, controller_width, controller_height, controller_bits);
  u8g2.drawXBMP( antenna_x, antenna_y, antenna_width, antenna_height, antenna_bits);
    
  redraw(255,readBatt(),ARC);
  //u8g2.sendBuffer();         // transfer internal memory to the display


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
  uint8_t rxBatt;
  uint8_t txBatt;
  uint8_t pipe; //Not really used, but might be relevant in the future?

  checkPaddles();
  uint8_t throttleVal = 3 + readStick(xPin, yPin, throttle_aPin, throttle_bPin); //-3 to 3 -> 0 to 6
  uint8_t steeringVal = 3 + readStick(xPin, yPin, steering_aPin, steering_bPin);

  uint8_t payload[2];
  payload[0] = speed << 6 | steeringVal << 3 | throttleVal; ///JOIN THE VALUES into one byte
  payload[1] = ARC;



 
  if(millis()-lastTXtime > TXPERIOD || payload[0] != lastPayload) 
  { 
    //Serial1.write(payload); //SEND IT!
    bool report = radio.write(&payload, 2);    // transmit & save the report
    if (report) 
        {
            ARC = radio.getARC(); //get automatic retransmission count.. to be used as a rough RSSI/link quality estimate.
            // get incoming ACK payload (should be one byte)
            if (radio.available(&pipe)) radio.read(&rxBatt,1);
            else Serial.println(F(" Recieved: an empty ACK packet?!")); // empty ACK packet received   
            lastTXtime = millis();
            lastPayload = payload[0];
        }
    else //NO CONNECTION:
        {
          ARC = 16;
          rxBatt = 255; 
        }
  }

  if(millis()-lastRedraw > FRAMERATE)
  {
      txBatt = readBatt();
      redraw(rxBatt,txBatt,ARC);
      lastRedraw=millis();   
  }
}
//Voltage divider: None.
#define V_BATTMAX 4.2
#define V_BATTMIN 3.0
#define V_ADCMAX 5.0
#define ADCMAX 1023.0
#define V_DIVIDER_MAX_OUT 4.2
#define DIVIDER_FACTOR 1.0
#define V_CALIBATED_OFFSET 0.09
#define N_MEASUREMENTS 64

uint8_t readBatt(){
  //Do a bunch of ADC measurements and convert them to average Vbatt, append Vbatt to report
  unsigned long ADCSum = 0;
  for (int i=0; i<N_MEASUREMENTS; i++) ADCSum+=analogRead(ADCpin); 
  unsigned int ADCavg = ADCSum/N_MEASUREMENTS;
  float vBatt = (float)ADCavg*V_ADCMAX/ADCMAX/DIVIDER_FACTOR+V_CALIBATED_OFFSET; 
          //Vbatt minimum = 3.0, VbattMaximum = 4.2
  //Serial.println(vBatt);
  return uint8_t(((vBatt - V_BATTMIN) * 100.0 / (V_BATTMAX - V_BATTMIN))); //calculate battery percentage
}


//WORST CASE RUNTIME: >20mS
void checkPaddles()
{
  static bool lastState_fasterPaddle = HIGH;
  static bool lastState_slowerPaddle = HIGH;

  if(!digitalRead(fasterPaddlePin) || !digitalRead(slowerPaddlePin) ) delay(20); //debounce

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

}

int readStick(int X, int Y, int A, int B){

static int aState;
static int bState;

pinMode(X,OUTPUT);
//digitalWrite(X,LOW); //perhaps redundant if state never changes?
aState = digitalRead(A);
bState = digitalRead(B);
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

/*
RX |||||||||||||||      75%
TX |||||||||||          55%
*/
void redraw(uint8_t _rxBatt, uint8_t _txBatt, uint8_t _ARC)
{
  static uint8_t lastRxPercent;
  static uint8_t lastTxPercent;
  static uint8_t lastARC;
  
  /*
  Serial.print(millis());
  Serial.print(',');

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
  if(_rxBatt != lastRxPercent || _txBatt != lastTxPercent || _ARC != lastARC){

        u8g2.setDrawColor(0);
        u8g2.drawBox(car_width+4,1,100,12); //clear RX percentage bar
        u8g2.drawBox(controller_width+4,19,antenna_x-(controller_width+4),12); //clear TX percentage bar
        u8g2.drawBox(88,19,40,12); //clear ARC bar
        
    //Boxes:
    u8g2.setDrawColor(1);
    if(_rxBatt<101) u8g2.drawBox(24,1,_rxBatt,12); //RX
    u8g2.drawBox(controller_width+4,19,_txBatt/3,12);             //TX
    if(_ARC<16) u8g2.drawBox(88,19,map(_ARC,15,0,0,40),12);   //ARC
    
    //Text:
    u8g2.setDrawColor(2);

        //RX
        u8g2.setCursor(60,12);
        if(_rxBatt>100) u8g2.print("?");
        else {
          u8g2.print(_rxBatt);
          u8g2.print("%");
        }
        //TX
        u8g2.setCursor(30,30);
        u8g2.print(_txBatt);
        u8g2.print("%");

        //ARC
        u8g2.setCursor(104,30);
      if(_ARC<16) u8g2.print(_ARC);
      else u8g2.print("?");


    lastRxPercent = _rxBatt;
    lastTxPercent = _txBatt;
    lastARC = _ARC;
    u8g2.sendBuffer();         // transfer internal memory to the display
  }
 // Serial.println(millis());

}

void nrf24Setup()
{
    radio.setDataRate(RF24_250KBPS);
    radio.setPALevel(RF24_PA_MAX);     // RF24_PA_MAX is default.
    radio.setRetries(5,15); //delay: The default value of 5 means 1500us (5 * 250 + 250), count:  The default/maximum is 15. Use 0 to disable the auto-retry feature all together.
    // to use ACK payloads, we need to enable dynamic payload lengths (for all nodes)
    radio.enableDynamicPayloads();    // ACK payloads are dynamically sized
    // Acknowledgement packets have no payloads by default. We need to enable
    // this feature for all nodes (TX & RX) to use ACK payloads.
    radio.enableAckPayload();
    // set the TX address of the RX node into the TX pipe
    radio.openWritingPipe(address[radioNumber]);     // using pipe 0
    // set the RX address of the TX node into a RX pipe
    radio.openReadingPipe(1, address[!radioNumber]); // using pipe 1
}
