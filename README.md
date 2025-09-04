# dirtCrusher
Reverse engineering and replacing most of the innards of a LEGO dirt crusher

V0.5 Used the HC-12 serial radio module.

[V1](V1_NRF24) Implements a 2 byte per frame radio link with a pair of NRF24L01+ modules.

First payload byte is transmitter readout, second payload byte is the automatic retransmission count, used as a makeshift rssi for the OSD on the analog FPV feed.

[V2](V2_LORA) Used a pair of SX1276 Lora modules.

[V3](V3_ELRS) Uses a pair of ELRS radio modules

Also: The transmitters have a built in lipo battery that charges with a USB-C cable... And a little OLED to show telemetry that looks like this:
![OLED simulation](/transmitter/GFX/oled_simulation.png)
