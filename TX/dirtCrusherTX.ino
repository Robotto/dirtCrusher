
const int HC12_SETpin = 3; //command pin
bool HC12_commandMode = false;

const int xPin = 4;
const int yPin = 5;
const int throttle_aPin = 6;
const int throttle_bPin = 7;

const int steering_aPin = 10;
const int steering_bPin = 16;

const int fasterPaddlePin = 14;
const int slowerPaddlePin = 15;

uint8_t rx=255;

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
  pinMode(xPin,INPUT);
  pinMode(yPin,INPUT);
  pinMode(throttle_aPin,INPUT_PULLUP);
  pinMode(throttle_bPin,INPUT_PULLUP);
  pinMode(steering_aPin,INPUT_PULLUP);
  pinMode(steering_aPin,INPUT_PULLUP);
  pinMode(fasterPaddlePin,INPUT_PULLUP);
  pinMode(slowerPaddlePin,INPUT_PULLUP);  

  digitalWrite(xPin,LOW);
  digitalWrite(yPin,LOW);
}


void loop() {
  uint8_t throttleVal = 3 + readStick(xPin, yPin, throttle_aPin, throttle_bPin); //-3 to 3 -> 0 to 6
  uint8_t steeringVal = 3 + readStick(xPin, yPin, steering_aPin, steering_bPin);

  uint8_t payload = speed << 6 | steeringVal << 3 | throttleVal; ///JOIN THE VALUES into one byte

  Serial1.write(payload); //SEND IT!

  delay(40);
  if(Serial1.available()) rx = Serial1.read();
}

//WORST CASE RUNTIME: 40mS
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
