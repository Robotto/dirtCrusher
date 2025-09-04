#define ADC_MIN 0
#define ADC_MAX 1023


// ELRS 3.x (ESP8266 based TX module): with thanks to r-u-t-r-A (https://github.com/r-u-t-r-A/STM32-ELRS-Handset/tree/v4.5)
//  1 : Set Lua [Packet Rate]= 0 - 50Hz / 1 - 100Hz Full / 2- 150Hz / 3 - 250Hz / 4 - 333Hz Full / 5 - 500Hz
//  2 : Set Lua [Telem Ratio]= 0 - Std / 1 - Off / 2 - 1:128 / 3 - 1:64 / 4 - 1:32 / 5 - 1:16 / 6 - 1:8 / 7 - 1:4 / 8 - 1:2 / 9 - Race
//  3 : Set Lua [Switch Mode]=0 -> Hybrid;Wide
//  4 : Set Lua [Model Match]=0 -> Off;On
//  5 : Set Lua [TX Power]=0 Submenu
// 6 : Set Lua [Max Power]=0 - 10mW / 1 - 25mW / 2 - 50mW /3 - 100mW/4 - 250mW  // dont force to change, but change after reboot if last power was greater
// 7 : Set Lua [Dynamic]=0 - Off / 1 - Dyn / 2 - AUX9 / 3 - AUX10 / 4 - AUX11 / 5 - AUX12
// 8 : Set Lua [VTX Administrator]=0 Submenu
// 9 : Set Lua [Band]=0 -> Off;A;B;E;F;R;L
// 10:  Set Lua [Channel]=0 -> 1;2;3;4;5;6;7;8
// 11 : Set Lua [Pwr Lvl]=0 -> -;1;2;3;4;5;6;7;8
// 12 : Set Lua [Pitmode]=0 -> Off;On 
// 13 : Set Lua [Send VTx]=0 sending response for [Send VTx] chunk=0 step=2
// 14 : Set Lua [WiFi Connectivity]=0 Submenu
// 15 : Set Lua [Enable WiFi]=0 sending response for [Enable WiFi] chunk=0 step=0
// 16 : Set Lua [Enable Rx WiFi]=0 sending response for [Enable Rx WiFi] chunk=0 step=2
// //17 : Set Lua [BLE Joystick]=0 sending response for [BLE Joystick] chunk=0 step=0  // not on ESP8266??
// //    Set Lua [BLE Joystick]=1 sending response for [BLE Joystick] chunk=0 step=3
// //    Set Lua [BLE Joystick]=2 sending response for [BLE Joystick] chunk=0 step=3
// 17: Set Lua [Bind]=0 -> 

// 3 Default Settings
#define SETTING_1_PktRate 3 // 250Hz (-108dB)
#define SETTING_1_Power 3   // 100mW
#define SETTING_1_Dynamic 1 // Dynamic power on

#define SETTING_2_PktRate 0 // 50Hz (-115dB)
#define SETTING_2_Power 3   // 100mW
#define SETTING_2_Dynamic 0 // Dynamic power off

enum chan_order
{
    AILERON,
    ELEVATOR,
    THROTTLE,
    RUDDER,
    AUX1, // (CH5)  ARM switch for Expresslrs
    AUX2, // (CH6)  angel / airmode change
    AUX3, // (CH7)  flip after crash
    AUX4, // (CH8)
    AUX5, // (CH9)
    AUX6, // (CH10)
    AUX7, // (CH11)
    AUX8, // (CH12)
};
