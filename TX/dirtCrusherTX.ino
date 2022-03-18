
const int HC12_SETpin = 8; //command pin
bool HC12_commandMode = false;

const int xPin = 14; //ORANGE
const int yPin = 5; //GREEN
const int throttle_aPin = A3; //yellow
const int throttle_bPin = A2; //WHITE

const int steering_aPin = 7; //yellow
const int steering_bPin = 4; //white

const int fasterPaddlePin = 6; //blue
const int slowerPaddlePin = 15; //pink

uint8_t rx=255; //TODO: receive battery telemetry from dirtCrusher

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

  if(Serial1.available()) rx = Serial1.read();

  if(millis()-lastTXtime > TXPERIOD) {
    Serial1.write(payload); //SEND IT!
    lastTXtime=millis();
  }
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
