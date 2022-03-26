#include <U8g2lib.h>
U8G2_SSD1306_128X32_UNIVISION_F_HW_I2C u8g2(U8G2_R0); 

//https://forum.arduino.cc/t/u8glib-and-bitmap-creation-display/148125/2


//CAR ICON:
#define car_width 20
#define car_height 15
static const unsigned char car_bits[] U8X8_PROGMEM = {
   0x3c, 0xcf, 0x03, 0xe0, 0x7f, 0x00, 0xf0, 0xff, 0x00, 0x38, 0xc0, 0x00,
   0x18, 0xc0, 0x01, 0x1f, 0x80, 0x0f, 0xfe, 0xff, 0x07, 0xfe, 0xff, 0x07,
   0xe6, 0x7f, 0x06, 0xc2, 0x3f, 0x04, 0xe6, 0x7f, 0x06, 0xfe, 0xff, 0x07,
   0xfc, 0xff, 0x03, 0x1c, 0x80, 0x03, 0x1c, 0x80, 0x03 };

//CONTROLLER ICON:
#define controller_width 20
#define controller_height 14
static const unsigned char controller_bits[] U8X8_PROGMEM = {
   0x00, 0x02, 0x00, 0x00, 0x02, 0x00, 0x00, 0x02, 0x00, 0xf0, 0xff, 0x00,
   0x0c, 0x00, 0x03, 0x02, 0x00, 0x04, 0x01, 0x00, 0x08, 0x11, 0x40, 0x08,
   0x39, 0xe0, 0x08, 0x11, 0x40, 0x08, 0x01, 0x00, 0x08, 0x02, 0x00, 0x04,
   0xc4, 0x1f, 0x02, 0x38, 0xe0, 0x01 };

const int HC12_SETpin = 8; //command pin
bool HC12_commandMode = false;

const int ADCpin = A0;

const int xPin = 14; //ORANGE
const int yPin = 5; //GREEN
const int throttle_aPin = A3; //yellow
const int throttle_bPin = A2; //WHITE

const int steering_aPin = 7; //yellow
const int steering_bPin = 4; //white

const int fasterPaddlePin = 6; //blue
const int slowerPaddlePin = 15; //pink

uint8_t rxBatt=111; //battery status should go from 0-100%, but integer underflows will make the data weird.
uint8_t txBatt=255; 
const unsigned long telemetryTimeout = 1000;
unsigned long lastTelemetryRXtime = 0;

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
//  Serial.begin(115200);
  Serial1.begin(9600);

  u8g2.begin();
  u8g2.setDisplayRotation(U8G2_R2);
  u8g2.setFont(u8g_font_unifont); //11 pixels high?
  u8g2.setFontMode(1); //transparent font
  

  redraw();
  //u8g2.sendBuffer();         // transfer internal memory to the display

  //pinMode(HC12_SETpin, OUTPUT);
  //digitalWrite(HC12_SETpin, !commandMode);

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

unsigned int TXPERIOD = 300; //300 ms between transmits seems kinda high, but code will also transmit immediately on state change.
unsigned int FRAMERATE = 1000;
unsigned long lastTXtime = 0;
unsigned long lastRedraw = 0;
uint8_t lastPayload;
void loop() {

  checkPaddles();
  uint8_t throttleVal = 3 + readStick(xPin, yPin, throttle_aPin, throttle_bPin); //-3 to 3 -> 0 to 6
  uint8_t steeringVal = 3 + readStick(xPin, yPin, steering_aPin, steering_bPin);

  uint8_t payload = speed << 6 | steeringVal << 3 | throttleVal; ///JOIN THE VALUES into one byte

  if(Serial1.available()) 
  {
    rxBatt = Serial1.read();
    lastTelemetryRXtime=millis();
  }

  if(millis()-lastTelemetryRXtime > telemetryTimeout) rxBatt = 111;

  if(millis()-lastTXtime > TXPERIOD || payload != lastPayload) { 
    Serial1.write(payload); //SEND IT!
    lastTXtime=millis();
    lastPayload=payload;
  }

  if(millis()-lastRedraw > FRAMERATE){
  txBatt = readBatt();
  redraw();
  lastRedraw=millis();
  }



}
//Voltage divider: VBATT -> 51K <-ADC-> 75K -> GND
#define V_BATTMAX 4.2
#define V_BATTMIN 3.0
#define V_ADCMAX 5.0
#define ADCMAX 1023.0
#define V_DIVIDER_MAX_OUT 4.2
#define DIVIDER_FACTOR 1.0
#define V_CALIBATED_OFFSET 0.0
#define N_MEASUREMENTS 16

uint8_t readBatt(){
  //Do a bunch of ADC measurements and convert them to average Vbatt, append Vbatt to report
  unsigned long ADCSum = 0;
  for (int i=0; i<N_MEASUREMENTS; i++) ADCSum+=analogRead(ADCpin); 
  unsigned int ADCavg = ADCSum/N_MEASUREMENTS;
  float vBatt = (float)ADCavg*V_ADCMAX/ADCMAX/DIVIDER_FACTOR+V_CALIBATED_OFFSET; 
          //Vbatt minimum = 7.0, VbattMaximum = 8.4
  return uint8_t(((vBatt - V_BATTMIN) * 100.0 / (V_BATTMAX - V_BATTMIN))); //calculate battery percentage
}


//WORST CASE RUNTIME: >20mS
int checkPaddles()
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
void redraw()
{
  static uint8_t lastRxPercent;
  static uint8_t lastTxPercent;
  
  if(rxBatt != lastRxPercent || txBatt != lastTxPercent){
    u8g2.clearBuffer();					// clear the internal memory
/*
    u8g2.setCursor(0,13);
    u8g2.print("RX:");
    u8g2.setCursor(0,30);
    u8g2.print("TX:");  
*/
    u8g2.drawXBMP( 0, 0, car_width, car_height, car_bits);
    u8g2.drawXBMP( 0, car_height+3, controller_width, controller_height, controller_bits);

    
    u8g2.setDrawColor(1);
    if(rxBatt<110) u8g2.drawBox(28,0,rxBatt,15);
    u8g2.drawBox(28,17,txBatt,15);
    u8g2.setDrawColor(2);
    u8g2.setCursor(60,13);
    if(rxBatt>110) u8g2.print("?");
    else {
      u8g2.print(rxBatt);
      u8g2.print("%");
    }
    u8g2.setCursor(60,30);
    u8g2.print(txBatt);
    u8g2.print("%");
    lastRxPercent = rxBatt;
    lastTxPercent = txBatt;
    u8g2.sendBuffer();         // transfer internal memory to the display
  }
}

