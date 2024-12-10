/*
// Simple Arduino trasmisster
// Arduino Nano
// ELRS 2.4G TX moduel
// Custom PCB from JLCPCB
// const float codeVersion = 0.92; // Software revision
// https://github.com/kkbin505/Arduino-Transmitter-for-ELRS

 * This file is part of Simple TX
 *
 * Simple TX is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * Simple TX is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with Cleanflight.  If not, see <http://www.gnu.org/licenses/>.
 */

#include <Arduino.h>

#include "config.h"
#include "crsf.h"

//#define DEBUG // if not commented out, Serial.print() is active! For debugging only!!
//#define GIMBAL_CALIBRATION // if not commented out, Serial.print() is active! For debugging only!!

int Throttle_value = 0;
int Rudder_value = 0;
int previous_throttle = 191;

int loopCount = 0; // for ELRS seeting


float batteryVoltage;

int currentPktRate = 0;
int currentPower = 0;
int currentDynamic = 0;
int currentSetting = 0;
int stickMoved = 0;
int stickInt = 0;
uint32_t stickMovedMillis = 0;

uint32_t currentMillis = 0;

uint8_t crsfPacket[CRSF_PACKET_SIZE];
uint8_t crsfCmdPacket[CRSF_CMD_PACKET_SIZE];
int16_t rcChannels[CRSF_MAX_CHANNEL];
uint32_t crsfTime = 0;

CRSF crsfClass;

bool calStatus=false;

uint8_t aux2cnt = 0;

bool checkStickMove(){
    // check if stick moved, warring after 10 minutes
    if(abs(previous_throttle - rcChannels[THROTTLE]) < 30){
        stickMoved = 0;
        //Serial.println(abs(previous_throttle - rcChannels[THROTTLE]));
    }else{
        previous_throttle = rcChannels[THROTTLE];
        stickMovedMillis = millis();
        stickMoved = 1;
    }

    if (millis() - stickMovedMillis > STICK_ALARM_TIME){
       // Serial.println((millis() - stickMovedMillis));
        return false; //TODO: CHANGE THIS BACK!!
    }else{
        return false;
    }
}

void setup()
{
    // inialize rc data
    for (uint8_t i = 0; i < CRSF_MAX_CHANNEL; i++) {
        rcChannels[i] = CRSF_DIGITAL_CHANNEL_MIN;
    }

    pinMode(DIGITAL_PIN_SWITCH_ARM, INPUT_PULLUP);
    pinMode(DIGITAL_PIN_SWITCH_AUX2, INPUT_PULLUP);
    pinMode(DIGITAL_PIN_SWITCH_AUX3, INPUT_PULLUP);
    if (DIGITAL_PIN_SWITCH_AUX4 != 0){
      pinMode(DIGITAL_PIN_SWITCH_AUX4, INPUT_PULLUP);
    }
    batteryVoltage = 0.0;

        Serial.begin(115200);
        crsfClass.begin(); //Set to use Serial1 on the Leonardo...

}

void loop()
{
    uint32_t currentMicros = micros();

    // Read Voltage
    batteryVoltage = analogRead(VOLTAGE_READ_PIN) / VOLTAGE_SCALE; // 98.5
    //Serial.print("batteryVoltage:");
    //Serial.print(batteryVoltage);
    //Serial.print("v ");
    if (batteryVoltage < WARNING_VOLTAGE && batteryVoltage >= BEEPING_VOLTAGE) {
    //    blinkLED(DIGITAL_PIN_LED, 500);
    }else if(batteryVoltage < BEEPING_VOLTAGE && batteryVoltage >= ON_USB){
    //    blinkLED(DIGITAL_PIN_LED, 250);
    }else if(currentSetting == 3){
     //   blinkLED(DIGITAL_PIN_LED, 100);  // Bind (fast flash)
    }else if(currentSetting == 4){
     //   blinkLED(DIGITAL_PIN_LED, 1000); // Wifi (slow flash)
    }

    if (checkStickMove() == true){
    //    blinkLED(DIGITAL_PIN_LED, 100);
    }

    
    
Throttle_value = (uint16_t)((1.0+sin((float)millis()/1000.0))*ADC_MAX);
   // Throttle_value = constrain(Throttle_value, CRSF_DIGITAL_CHANNEL_MIN, CRSF_DIGITAL_CHANNEL_MAX); 
    Serial.print(Throttle_value);

    Rudder_value = (uint16_t)((1.0+cos((float)millis()/1000.0))*ADC_MAX);
    Serial.println(Rudder_value);

    // rcChannels[AILERON] = map(Aileron_value, 1023 - ANALOG_CUTOFF, ANALOG_CUTOFF, CRSF_DIGITAL_CHANNEL_MIN, CRSF_DIGITAL_CHANNEL_MAX);   // reverse
    // rcChannels[ELEVATOR] = map(Elevator_value, 1023 - ANALOG_CUTOFF, ANALOG_CUTOFF, CRSF_DIGITAL_CHANNEL_MIN, CRSF_DIGITAL_CHANNEL_MAX); // reverse
    // rcChannels[THROTTLE] = map(Throttle_value, 1023 - ANALOG_CUTOFF, ANALOG_CUTOFF, CRSF_DIGITAL_CHANNEL_MIN, CRSF_DIGITAL_CHANNEL_MAX); // reverse
    // rcChannels[RUDDER] = map(Rudder_value, ANALOG_CUTOFF, 1023 - ANALOG_CUTOFF, CRSF_DIGITAL_CHANNEL_MIN, CRSF_DIGITAL_CHANNEL_MAX);
    rcChannels[AILERON]   = 0; 
    rcChannels[ELEVATOR]  = 0;
    rcChannels[THROTTLE]  = map(Throttle_value, ADC_MIN, ADC_MAX, CRSF_DIGITAL_CHANNEL_MIN, CRSF_DIGITAL_CHANNEL_MAX);
    rcChannels[RUDDER]    = map(Rudder_value,   ADC_MIN, ADC_MAX, CRSF_DIGITAL_CHANNEL_MIN, CRSF_DIGITAL_CHANNEL_MAX);

    if(stickInt=0){
        previous_throttle=rcChannels[THROTTLE];
        stickInt=1;
    }
    if (loopCount == 0) {
        // Check if sticks are held in specific position on startup (bind/wifi/packet rate select)
        //selectSetting();
    }

    if (currentMicros > crsfTime) {
        /*
        Serial.println("DEBUG");
        Serial.print(" AILERON:");
        Serial.print(rcChannels[AILERON]);
        Serial.print(" ELEVATOR:");
        Serial.print(rcChannels[ELEVATOR] );
        Serial.print(" THROTTLE:");
        Serial.print(rcChannels[THROTTLE]);
        Serial.print(" RUDDER:");
        Serial.print(rcChannels[RUDDER] );
        Serial.print(" stickstatus:");
        Serial.print(stickMoved);
        Serial.print(" previous_throttle:");
        Serial.print(previous_throttle);
        Serial.println(); 
        */
            if (loopCount <= 500) { // repeat 500 packets to build connection to TX module
                // Build commond packet
                crsfClass.crsfPrepareDataPacket(crsfPacket, rcChannels);
                crsfClass.CrsfWritePacket(crsfPacket, CRSF_PACKET_SIZE);
                loopCount++;
            }

            if (loopCount > 500 && loopCount <= 505) { // repeat 5 packets to avoid bad packet, change rate setting
                // Build commond packet
                if (currentSetting == 1 || currentSetting == 2) {
                    crsfClass.crsfPrepareCmdPacket(crsfCmdPacket, ELRS_PKT_RATE_COMMAND, currentPktRate);
                    crsfClass.CrsfWritePacket(crsfCmdPacket, CRSF_CMD_PACKET_SIZE);
                } else if (currentSetting == 3) {
                    crsfClass.crsfPrepareCmdPacket(crsfCmdPacket, ELRS_BIND_COMMAND, ELRS_START_COMMAND);
                    crsfClass.CrsfWritePacket(crsfCmdPacket, CRSF_CMD_PACKET_SIZE);
                } else if (currentSetting == 4) {
                    crsfClass.crsfPrepareCmdPacket(crsfCmdPacket, ELRS_WIFI_COMMAND, ELRS_START_COMMAND);
                    crsfClass.CrsfWritePacket(crsfCmdPacket, CRSF_CMD_PACKET_SIZE);
                }
                loopCount++;
            } else if (loopCount > 505 && loopCount <= 510) { // repeat 5 packets to avoid bad packet, change TX power level
                if (currentSetting == 1 || currentSetting == 2) {
                    crsfClass.crsfPrepareCmdPacket(crsfCmdPacket, ELRS_POWER_COMMAND, currentPower);
                    crsfClass.CrsfWritePacket(crsfCmdPacket, CRSF_CMD_PACKET_SIZE);
                }
                loopCount++;
            } else if (loopCount > 510 && loopCount <= 515) { // repeat 5 packets to avoid bad packet, change TX dynamic power setting
                if (currentSetting == 1 || currentSetting == 2) {
                    crsfClass.crsfPrepareCmdPacket(crsfCmdPacket, ELRS_DYNAMIC_POWER_COMMAND, currentDynamic);
                    crsfClass.CrsfWritePacket(crsfCmdPacket, CRSF_CMD_PACKET_SIZE);
                }
                loopCount++;
            } else {
                crsfClass.crsfPrepareDataPacket(crsfPacket, rcChannels);
                crsfClass.CrsfWritePacket(crsfPacket, CRSF_PACKET_SIZE);
            }

        crsfTime = currentMicros + CRSF_TIME_BETWEEN_FRAMES_US;
    }
}

ISR(TIMER1_COMPA_vect) { // leave this alone
    static boolean state = true;

    TCNT1 = 0;

    if (state) { // start pulse
        digitalWrite(ppmPin, onState);
        OCR1A = PULSE_LENGTH * 2;
        state = false;
    } else { // end pulse and calculate when to start the next pulse
        static byte cur_chan_numb;
        static unsigned int calc_rest;

        digitalWrite(ppmPin, !onState);
        state = true;

        if (cur_chan_numb >= CHANNEL_NUMBER) {
            cur_chan_numb = 0;
            calc_rest = calc_rest + PULSE_LENGTH; //
            OCR1A = (FRAME_LENGTH - calc_rest) * 2;
            calc_rest = 0;
        } else {
            OCR1A = (map(rcChannels[cur_chan_numb], CRSF_DIGITAL_CHANNEL_MIN, CRSF_DIGITAL_CHANNEL_MAX, 1000, 2000) - PULSE_LENGTH) * 2;
            calc_rest = calc_rest + map(rcChannels[cur_chan_numb], CRSF_DIGITAL_CHANNEL_MIN, CRSF_DIGITAL_CHANNEL_MAX, 1000, 2000);
            cur_chan_numb++;
        }
    }
}
