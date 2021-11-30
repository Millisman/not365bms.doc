#!/bin/bash

#Ext crystal, 8+ MHz, 16K CK/14 CK + 0 ms
#Preserve EEPROM memory through the Chip Erase cycle
#Serial program downloading (SPI) enabled
#Brown-out detection disable
# -U lfuse:w:0xdf:m -U hfuse:w:0xd6:m -U efuse:w:0xff:m


avrdude -v -p m328p -c usbtiny -U lfuse:w:0xdf:m -U hfuse:w:0xd6:m -U efuse:w:0xff:m

avrdude -v -p m328p -c usbtiny -U flash:w:optiboot_m328_12M_115200_LEDB5_FLASHES_0.hex:i
