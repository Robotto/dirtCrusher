void setup() {
  Serial1.begin(1200);
  Serial.begin(115200);
}

uint8_t rx;

void loop(){
  if (Serial1.available()) {
    rx=Serial1.read();
  
  
  //Parse received data:
  int throttle = -3+(rx&0b00000111);
  int steering = (-3+((rx&0b00111000)>>3))*(-1); //-1 reverses steering direction.
  uint8_t speedFactor = (rx&0b11000000)>>6;

                      //    1-3                -3-3
  int throttlePWMdiff = speedFactor * 14 * throttle; // 127/(3*3) = 14.11
  uint8_t pwmVal = 128 + throttlePWMdiff;
  int mapVal = map(pwmVal,0,255,31,62);

  

  Serial.print("RX: "); Serial.println(rx,BIN);
  Serial.print("T:"); Serial.print(throttle); Serial.print(", S:"); Serial.print(steering); Serial.print(", X:"); Serial.println(speedFactor); 
  Serial.print("PWM: "); Serial.print(pwmVal); Serial.print(','); Serial.println(mapVal);
    }
}
