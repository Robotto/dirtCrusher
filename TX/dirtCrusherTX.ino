#include <U8g2lib.h>
U8G2_SSD1306_128X32_UNIVISION_F_HW_I2C u8g2(U8G2_R0); 

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

uint8_t rxBatt=0; 
uint8_t txBatt=255; 

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
  Serial1.begin(1200);

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

unsigned int TXPERIOD = 100; //10 transmits per second.. seems kinda low..
unsigned long lastTXtime=0;
void loop() {

  checkPaddles();
  uint8_t throttleVal = 3 + readStick(xPin, yPin, throttle_aPin, throttle_bPin); //-3 to 3 -> 0 to 6
  uint8_t steeringVal = 3 + readStick(xPin, yPin, steering_aPin, steering_bPin);

  uint8_t payload = speed << 6 | steeringVal << 3 | throttleVal; ///JOIN THE VALUES into one byte

  if(Serial1.available()) rxBatt = Serial1.read();


  if(millis()-lastTXtime > TXPERIOD) {
    Serial1.write(payload); //SEND IT!
    lastTXtime=millis();

    //also do things with battery percentages...
    txBatt = readBatt();
    redraw();
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
#define N_MEASUREMENTS 64

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
    u8g2.setCursor(0,13);
    u8g2.print("RX:");
    u8g2.setCursor(0,30);
    u8g2.print("TX:");  
    u8g2.setDrawColor(1);
    u8g2.drawBox(28,0,rxBatt,15);
    u8g2.drawBox(28,17,txBatt,15);
    u8g2.setDrawColor(2);
    u8g2.setCursor(60,13);
    u8g2.print(rxBatt);
    u8g2.print("%");
    u8g2.setCursor(60,30);
    u8g2.print(txBatt);
    u8g2.print("%");
    lastRxPercent = rxBatt;
    lastTxPercent = txBatt;
    u8g2.sendBuffer();         // transfer internal memory to the display
  }
}