//HC-12 AT Commander
//The HC-12 default baud rate is 9600

/*
while energized, pull Pin 5 (“SET”) low,
wait 40ms for command mode to engage
*/

#define MODE_SWITCHING_ENABLED

//const int SETpin = 8; //command pin on dirtcrusher TX
const int SETpin = 3; //command pin on dirtcrusher RX
bool commandMode = false;

void setup() {
	while(!Serial); //wait for serial connection from PC
  Serial.begin(115200);
  //Serial1.begin(2400);
  //Serial1.begin(9600);
  //Serial1.begin(1200);
  //Serial1.begin(19200);
  Serial1.begin(115200);
  
  pinMode(SETpin, OUTPUT);
  digitalWrite(SETpin, !commandMode);
#ifdef MODE_SWITCHING_ENABLED
  Serial.println("Now in RF mode. Switch modes with: \\");
#else  
 Serial.println("Now in RF mode. MODE SWITCHING DISABLED.");
#endif

//AT+RX gives current setup:
//OK+B115200
//OK+RC127
//OK+RP:+20dBm
//OK+FU3

//AT+V giver firmware version. make sure versions match between receiver and transmitter

}

char rx;

void loop(){
  if (Serial1.available()) Serial.write(Serial1.read()); //dump data directly from HC-12 to PC

#ifndef MODE_SWITCHING_ENABLED
  if (Serial.available()) Serial1.write(Serial.read()); //dump data directly from PC to HC12

#else

  if (Serial.available()){
  	rx=Serial.read();

  	if(rx=='\\') {// \\ means '\'', since \ is the escape char
  		commandMode=!commandMode; //flip the bool
  		digitalWrite(SETpin,!commandMode); //set the pin (command mode is active low)
  		if(commandMode) Serial.println("Command mode set.");
  		else Serial.println("RF mode set.");
  		delay(50);
  	}
    
   else Serial1.write(rx);
  }
#endif
}
