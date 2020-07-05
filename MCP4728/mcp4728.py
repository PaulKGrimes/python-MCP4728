# mcp4728.py

from __future__ import print_function
from __future__ import division

import smbus
from math import ceil
from time import sleep

_defaultVDD = 5000
_BASE_ADDR = 0x60
_RESET = 0B00000110
_WAKE = 0B00001001
_UPDATE = 0B00001000
_MULTIWRITE = 0B01000000
_SINGLEWRITE = 0B01011000
_SEQWRITE = 0B01010000
_VREFWRITE = 0B10000000
_GAINWRITE = 0B11000000
_PWRDOWNWRITE = 0B10100000
_GENERALCALL = 0B00000000


def value_to_bytes(value, bits=16):
    """Convert a value in a string of 8 bit bytes, and return them in
    a list with the most significant byte first"""
    length = len(bin(value)) - 2
    num_bytes = int(ceil(length / 8.))

    if bits is None:
        pass
    else:
        if bits < length:
            raise ValueError("Value is too large to fit in requested number of bits")
        num_bytes = int(ceil(bits / 8.))

    return_bytes = []
    for b in range(num_bytes):
        b = num_bytes - b
        byte_value = value >> 8 * (b - 1)
        value = value - (byte_value << 8 * (b - 1))
        return_bytes.append(byte_value)

    return return_bytes


def bytes_to_value(bytelist):
    """Convert a sequence of bytes into an integer. Assumes most significant byte is first"""
    length = len(bytelist)
    value = 0
    for b, byte in enumerate(bytelist):
        bits = len(bin(byte)) - 2
        value += byte << (length - b) * bits

    return value


class MCP4728(object):
    """A class representing the MCP4728 device on the i2c bus"""

    def __init__(self, device_id=0x00):
        """Create a MCP4728 object"""
        self._bus = smbus.SMBus(1)
        self._device_id = device_id
        self._dev_address = (_BASE_ADDR | self._device_id)
        self._vdd = _defaultVDD
        self._values = [0, 0, 0, 0]
        self._int_vref = [0, 0, 0, 0]
        self._gains = [0, 0, 0, 0]
        self._power_down = [0, 0, 0, 0]
        self._values_ep = [0, 0, 0, 0]
        self._int_vref_ep = [0, 0, 0, 0]
        self._gains_ep = [0, 0, 0, 0]
        self._power_down_ep = [0, 0, 0, 0]

        self.update_status()

    def reset(self):
        """General reset of MCP4728 - EEProm values will be loaded to input register"""
        return self._simple_command(_RESET)

    def wake(self):
        """General wake-up call of MCP4728"""
        return self._simple_command(_WAKE)

    def update(self):
        """General software update of MCP4728 - All DAC outputs update"""
        return self._simple_command(_UPDATE)

    def set_value_all(self, values):
        """Write input register values to each channel using fast_write method.

        Parameters:
            values: list: of four integers in range 0-4095"""
        if len(values) != 4:
            raise ValueError("Must pass four values to be written")
        self._values = values
        return self.fast_write()

    def set_value(self, channel, value):
        """Write input register value to specified channel using fast_write method

        parameters:
            channel: int 0-3: input channel
            value: int 0-4095: value to write"""
        self._values[channel] = value
        return self.fast_write()

    def eeprom_write_all(self):
        """Write all current values to each channel using Sequential Write method.
        This will update both the input register and EEProm stored values"""
        self.seq_write()
        self.update_status(invert_eeprom=True)

    def eeprom_write(self, channel):
        """Write current values to EEProm for channel using single_write method.
        Will write all output values, Vref, PowerDown and Gain settings"""
        self.single_write(channel)
        self.update_status(invert_eeprom=True)

    def eeprom_reset(self):
        """Set all EEProm values to the factory default"""
        for n in range(4):
            self._values[n] = 0
            self._int_vref[n] = 1
            self._gains[n] = 0
            self._power_down[n] = 0

        self.eeprom_write_all()

    def fast_write(self):
        """FastWrite input register values - All DAC ouput update. refer to DATASHEET 5.6.1
        DAC Input and PowerDown bits update.
        No EEPROM update"""
        block = []

        for channel in range(len(self._values)):
            val_word = value_to_bytes(self._values[channel])
            block.append(val_word[0])
            block.append(val_word[1])

        self._bus.write_i2c_block_data(self._dev_address, block[0], block[1:])

    def single_write(self, channel):
        """SingleWrite input register and EEPROM with a DAC output update.
        refer to DATASHEET 5.6.4
        DAC Input, Gain, Vref and PowerDown bits update
        EEPROM is updated"""
        val_word = value_to_bytes(self._values[channel])

        first = _SINGLEWRITE | (channel << 1)
        second = self._int_vref[channel] << 7 | self._power_down[channel] << 5 | self._gains[channel] << 4 | val_word[0]
        third = val_word[1]

        print(bin(first), second, third)
        self._bus.write_i2c_block_data(self._dev_address, first, [second, third])

    def multi_write(self):
        """MultiWrite input register values - All DAC ouput update. refer to DATASHEET 5.6.2
        DAC Input, Gain, Vref and PowerDown bits update
        No EEPROM update"""
        block = []

        for channel in range(len(self._values)):
            val_word = value_to_bytes(self._values[channel])
            block.append(_MULTIWRITE | (channel << 1))
            block.append(
                self._int_vref[channel] << 7 | self._power_down[channel] << 5 | self._gains[channel] << 4 | val_word[0])
            block.append(val_word[1])

        self._bus.write_i2c_block_data(self._dev_address, block[0], block[1:])

    def seq_write(self, start_channel=0):
        """Sequential Write ALL input register values - All DAC ouput update. refer to DATASHEET 5.6.2
        DAC Input, Gain, Vref and PowerDown bits update
        EEPROM is updated.

        Parameters:
            start_channel :int: - first channel to write. Will write values from start_channel to 3"""
        block = [_SEQWRITE | start_channel << 1]

        for channel in range(start_channel, len(self._values)):
            val_word = value_to_bytes(self._values[channel])

            block.append(
                self._int_vref[channel] << 7 | self._power_down[channel] << 5 | self._gains[channel] << 4 | val_word[0])
            block.append(val_word[1])

        print(block)
        self._bus.write_i2c_block_data(self._dev_address, block[0], block[1:])

    def write_vref(self):
        """Write the voltage reference settings to the input register"""
        data = _VREFWRITE | self._int_vref[0] << 3 | self._int_vref[1] << 2 | self._int_vref[2] << 1 | self._int_vref[3]
        self._bus.write_byte(self._dev_address, data)

    def set_vref_all(self, values):
        """Set the voltage reference settings and write them to input registers

        Parameters:
            values - list of four :int: vref settings for each channel:
                            - 0: use vdd
                            - 1: use internal 2.048V reference."""
        if len(values) != 4:
            raise ValueError("Must pass four values")
        self._int_vref = values
        return self.write_vref()

    def set_vref(self, channel, value):
        """Write the voltage reference settings for one channel to input registers.

        Parameters:
            channel :int: - channel number to write to.
            value :int: - vref setting:
                            - 0: use vdd
                            - 1: use internal 2.048V reference."""
        self._int_vref[channel] = value
        return self.write_vref()

    def write_gain(self):
        """Write the gain settings to the input register
        """
        data = _GAINWRITE | self._gains[0] << 3 | self._gains[1] << 2 | self._gains[2] << 1 | self._gains[3]
        self._bus.write_byte(self._dev_address, data)

    def set_gain_all(self, values):
        """Write the gain setting of the internal reference to input registers

        Parameters:
            values - list of four :int: gain settings:
                            - 0: gain of 1
                            - 1: gain of 2.
        """
        if len(values) != 4:
            raise ValueError("Must pass four values")

        self._gains = values
        return self.write_gain()

    def set_gain(self, channel, value):
        """Write the gain setting for one channel to input registers

        Parameters:
            channel :int: - channel number to write to.
            value - :int: gain settings:
                            - 0: gain of 1
                            - 1: gain of 2."""
        self._gains[channel] = value
        return self.write_gain()

    def write_power_down(self):
        """Write the power down setting to the input register"""
        cmd = _PWRDOWNWRITE | self._power_down[0] << 2 | self._power_down[1]
        data = self._power_down[2] << 6 | self._power_down[3] << 4
        self._bus.write_byte_data(self._dev_address, cmd, data)

    def set_power_down_all(self, values):
        """Write the power down settings to input registers

        Parameters:
            values - list of four :int: power down settings:
                            - 0: channel on
                            - 1: channel off
        """
        if len(values) != 4:
            raise ValueError("Must pass four values")

        self._power_down = values
        return self.write_power_down()

    def set_power_down(self, channel, value):
        """Write the power down setting for one channel to input registers

        Parameters:
            channel :int: - channel number to write to.
            value - :int: power down settings:
                            - 0: channel on
                            - 1: channel off
        """
        self._power_down[channel] = value
        return self.write_power_down()

    @property
    def device_id(self):
        """Return the device ID"""
        return self._device_id

    def get_vref(self, channel):
        """Return the vref setting of a channel"""
        return self._int_vref[channel]

    def get_gain(self, channel):
        """Return the gain setting of a channel"""
        return self._gains[channel]

    def get_power_down(self, channel):
        """Return the power down setting of a channel"""
        return self._power_down[channel]

    def get_value(self, channel):
        """Return the set value of a channel"""
        return self._values[channel]

    def get_vref_ep(self, channel):
        """Return the vref setting of a channel"""
        return self._int_vref_ep[channel]

    def get_gain_ep(self, channel):
        """Return the gain setting of a channel"""
        return self._gains_ep[channel]

    def get_power_down_ep(self, channel):
        """Return the power down setting of a channel"""
        return self._power_down_ep[channel]

    def get_value_ep(self, channel):
        """Return the set value of a channel"""
        return self._values_ep[channel]

    def get_vdd(self):
        """Return the set value of vdd"""
        return self._vdd

    def get_vout(self, channel):
        """Return the current Vout of a channel in millivolts"""
        if self._int_vref[channel] == 1:
            vref = 2048
        else:
            vref = self._vdd

        vout = vref * self._values[channel] * (self._gains[channel] * self._int_vref[channel] + 1) / 4096

        return vout

    def set_vout(self, channel, vout):
        """Write a voltage value in millivolts to one channel"""
        if self._int_vref[channel] == 1:
            vref = 2048
        else:
            vref = self._vdd

        value = int((vout * 4095) / (vref * (self._gains[channel] * self._int_vref[channel] + 1)))

        if value > 4095:
            value = 4095
        if value < 0:
            value = 0

        self.set_value(channel, value)

    def set_vout_all(self, vouts):
        """Write voltage values to all channels, supplied as a list vout"""
        if len(vouts) != 4:
            raise ValueError("Must supply four voltages")
        values = [0, 0, 0, 0]
        for channel in range(4):
            if self._int_vref[channel] == 1:
                vref = 2048
            else:
                vref = self._vdd

            values[channel] = int(
                (vouts[channel] * 4095) / (vref * (self._gains[channel] * self._int_vref[channel] + 1)))

            if values[channel] > 4095:
                values[channel] = 4095
            if values[channel] < 0:
                values[channel] = 0

        self.set_value_all(values)

    def print_status(self):
        """Print the current values"""
        print("Device ID     :", self._device_id)
        print("Values        :", self._values)
        print("Values EEProm :", self._values_ep)
        print("Vref          :", self._int_vref)
        print("Vref EEProm   :", self._int_vref_ep)
        print("Gain          :", self._gains)
        print("Gain EEProm   :", self._gains_ep)
        print("Power         :", self._power_down)
        print("Power EEProm  :", self._power_down_ep)

    def update_status(self, invert_eeprom=False):
        """Get the current values from the MCP4728.
        For some reason, the eeprom return bits are inverted after a sequential write.

        Parameters:
            invert_eeprom - if True, invert the eeprom statuses."""
        status = self._bus.read_i2c_block_data(self._dev_address, 0x02, 24)

        for n in range(4):
            device_id = status[n * 6]
            channel = (device_id & 0b00110000) >> 4
            if channel != n:
                raise RuntimeError("Error reading status from MCP4728 device")
            hi_byte = status[n * 6 + 1]
            lo_byte = status[n * 6 + 2]
            self._int_vref[n] = (hi_byte & 0b10000000) >> 7
            self._gains[n] = (hi_byte & 0b00010000) >> 4
            self._power_down[n] = (hi_byte & 0b0110000) >> 5
            self._values[n] = ((hi_byte & 0b00001111) << 8) + lo_byte
            device_id = status[n * 6 + 3]
            channel = (device_id & 0b00110000) >> 4
            if channel != n:
                raise RuntimeError("Error reading status from MCP4728 device")
            if invert_eeprom:
                hi_byte = ~status[n * 6 + 4]
                lo_byte = ~status[n * 6 + 5]
            else:
                hi_byte = status[n * 6 + 4]
                lo_byte = status[n * 6 + 5]

            print("{:#010b} : {:d}".format(hi_byte, (hi_byte & 0b00001111) << 8))
            print("{:#010b} : {:d}".format(lo_byte, lo_byte))

            self._int_vref_ep[n] = (hi_byte & 0b10000000) >> 7
            self._gains_ep[n] = (hi_byte & 0b00010000) >> 4
            self._power_down_ep[n] = (hi_byte & 0b0110000) >> 5
            self._values_ep[n] = ((hi_byte & 0b00001111) << 8) + lo_byte

    def _simple_command(self, command):
        """Send a simple byte command to the SMBus"""
        self._bus.write_byte_data(self._dev_address, _GENERALCALL, command)
        return
