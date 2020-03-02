#!/usr/bin/python

from MCP4728 import MCP4728_address
import time

"""
NOTES:
It is strongly advisable to use three of any general purpose (non-I2C, non-SPI,
non-UART) GPIO pins in order to avoid unwanted interferences.  Don't forget to
use pull-up resistors.  Mind the difference between GPIO numbers used in this
script and PIN numbers.  You can specify either long address (x61) or short
address (1) as higher bits are automatically disregarded.
"""

# MCP_address(scl_gpio,sda_gpio,ldac_gpio)
dac = MCP4728_address(19,26,21)

# GET ADDRESS
# getaddress()
cur=dac.getaddress()
print('Old address: 0x{0:02X}'.format(cur))
time.sleep(1)

# SET ADDRESS
# setaddress(curent_address,new_address)
dac.setaddress(cur,1)
time.sleep(1)

cur=dac.getaddress()
print('New address: 0x{0:02X}'.format(cur))
