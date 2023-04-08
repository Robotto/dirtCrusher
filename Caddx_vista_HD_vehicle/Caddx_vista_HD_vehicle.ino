#include "drv8871.h"

const int steeringFeedbackPinA = 4;
const int steeringFeedbackPinB = 5;
const int steeringFeedbackPinC = 6;
int previousState = 0; //holds steering feedback position from last read.
DRV8871 steeringDriver(7,8);

              void  setup()  {
                    pinMode(steeringFeedbackPinA,INPUT_PULLUP);
                    pinMode(steeringFeedbackPinB,INPUT_PULLUP);
                    pinMode(steeringFeedbackPinC,INPUT_PULLUP);
                      
                  Serial.begin(115200);
              }
              
              int steering=0;

              void loop()  {
                  now  = millis();

                  int steeringRX = 0; //TODO: UPDATE STEERING DATA FROM RADIO LINK

                  previousState = readSteeringFeedback();
                  int steeringDelta = steering-previousState; 
                  if(steeringDelta<0) steeringDriver.forward();//need to go left
                  else if(steeringDelta>0) steeringDriver.reverse();//need to go right
                  else steeringDriver.brake(); //need to go nowhere
                  
              }
               
int readSteeringFeedback(){ //negative is turning left!
  int aState = digitalRead(steeringFeedbackPinA);
  int bState = digitalRead(steeringFeedbackPinB);
  int cState = digitalRead(steeringFeedbackPinC);
  if( aState & bState & cState ) return previousState; //non-discrete in-between-state with all pins high - Keep moving the steering.
  //Now we know the feedback is in a discrete state:
  if( bState ) return 2-cState+aState; //Steering feedback is positive (1,2,3)
  else if ( cState & !aState ) return 0; // A=0, B=0; C=1
  else return -3+aState+cState; //-3,-2,-1
}
