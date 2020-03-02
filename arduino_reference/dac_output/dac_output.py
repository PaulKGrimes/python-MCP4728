#!/usr/bin/python

from MCP4728 import MCP4728

# MCP4728(i2c_address)
dac = MCP4728(0x60)

# specify the voltage of a channel (1-4) using internal reference, voltage in absolute value, 0 <= V <= 4.095
# single_internal(channel_number,absolute_voltage,eeprom)
dac.single_internal(2,3,False) # don't write to EEPROM
dac.single_internal(3,3,True) # write to EEPROM

# specify the voltage of a channel (1-4) using external reference, voltage relative to VDD, 0 <= r <= 0.9998
# single_external(channel_number,relative_voltage,eeprom)
dac.single_external(2,0.5,False) # don't write to EEPROM
dac.single_external(3,0.5,True) # write to EEPROM

# specify the voltage of four channels using internal reference, voltages in absolute value, 0 <= V <= 4.095
# multiple_internal(absolute_voltages,eeprom)
dac.multiple_internal([1.5,2,2.5,3],False) # don't write to EEPROM
dac.multiple_internal([1.5,2,2.5,3],True) # write to EEPROM

# specify the voltage of four channels using external reference, voltages relative to VDD, 0 <= r <= 0.9998
# multiple_external(relative_voltages,eeprom)
dac.multiple_external([0.2,0.3,0.4,0.5],False) # don't write to EEPROM
dac.multiple_external([0.2,0.3,0.4,0.5],True) # write to EEPROM
