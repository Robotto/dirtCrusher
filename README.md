# dirtCrusher
Reverse engineering and replacing most of the innards og a LEGO dirt crusher

Current version implements a 2 byte per frame radio link with a pair of NRF24L01+ modules.

First payload byte is transmitter readout, second payload byte is the automatic retransmission count, used as a makeshift rssi for the OSD on the analog FPV feed.

Also: The transmitter has a built in lipo battery that charges with a USB-C cable... And a little OLED to show telemetry that looks like this:
![OLED simulation](/transmitter/GFX/oled_simulation.png)
