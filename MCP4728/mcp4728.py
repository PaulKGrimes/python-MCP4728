# mcp4728.py

import smbus

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
_GAINWRITE = 0B11000000

class MCP4728(object):
    """A class representing the MCP4728 device on the i2c bus"""
    def __init__(self, deviceID = 0x00):
        """Create a MCP4728 object"""
        self._bus = smbus.SMBus(1)
        self._deviceID = deviceID
        self._dev_address = (_BASE_ADDR | self._deviceID)
        self._vdd = _defaultVDD
        self._values = [None, None, None, None]
        self._int_vref = [None, None, None, None]
        self._gains = [None, None, None, None]
        self._power_down = [None, None, None, None]
        self._values_ep = [None, None, None, None]
        self._int_vref_ep = [None, None, None, None]
        self._gains_ep = [None, None, None, None]
        self._power_down_ep = [None, None, None, None]


    def begin(self):
        """Start the I2C connection and get current values of MCP4728"""
        Wire.begin()
        self.get_status()

    def reset(self):
        """General reset of MCP4728 - EEProm values will be loaded to input register"""
        return _simple_command(_RESET)

    def wake(self):
        """General wake-up call of MCP4728"""
        return _simple_command(_WAKE)

    def update(self):
        """General software update of MCP4728 - All DAC outputs update"""
        return _simple_command(_UPDATE)

    def analog_write_all(self, values):
        """Write input register values to each channel using fastwrite method.

        Parameters:
            values: list: of four integers in range 0-4095"""
        if len(value) != 4:
            raise ValueError("Must pass four values to be written")
        self._values = values
        return fast_write()

    def analog_write(self, channel, value):
        """Write input register value to specified channel using fastwrite method

        parameters:
            channel: int 0-3: input channel
            value: int 0-4095: value to write"""
        self._values[channel] = value
        return fast_write()

    def eeprom_write_all(self, values):
        """Write values to each channel using Sequential Write method.
        This will update both the input register and EEProm stored values

        Parameters:
            value<n>: int: 0-4095"""
        if len(values) != 4:
            raise ValueError("Must pass four values to write to EEProm")
        self._values_ep = values
        self._values = self._values_ep

        return self.seq_write()

    def eeprom_write(self):
        """Write all current values to EEProm using seq_write method.
        Will write all output values, Vref, PowerDown and Gain settings"""
        return self.seq_write()

    def eeprom_reset(self):
        """Set all EEProm values to the factory default"""
        for n in range(4):
            self._values[n] = 0
            self._int_vref[n] = 1
            self._gains[n] = 0
            self._power_down[n] = 0

        self.seq_write()

    def set_vref_all(self, values):
        """Write the voltage reference settings to input registers"""
        if len(values) != 4:
            raise ValueError("Must pass four values")
        self._int_vref = values
        return self.write_vref()

    def set_vref(self, channel, value):
        """Write the voltage reference settings for one channel to input registers"""

        self._int_vref[channel] = value
        return self.write_vref()

    def set_gain_all(self, value1, value2, value3, value4):
        """Write the gain setting to input registers"""
        self._gains[0] = value1
        self._gains[1] = value2
        self._gains[2] = value3
        self._gains[3] = value4
        return self.write_gain()

    def set_gain(self, channel, value):
        """Write the gain setting for one channel to input registers"""
        self._gains[channel] = value
        return self.write_gain()

    def set_power_down_all(self, value1, value2, value3, value4):
        """Write the gain setting to input registers"""
        self._power_down[0] = value1
        self._power_down[1] = value2
        self._power_down[2] = value3
        self._power_down[3] = value4
        return self.write_power_down()

    def set_power_down(self, channel, value):
        """Write the gain setting for one channel to input registers"""
        self._power_down[channel] = value
        return self.write_power_down()

    @property
    def deviceID(self):
        """Return the device ID"""
        return self._deviceID

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
        if self._int_vref[channel] = 1:
            vref = 2048
        else:
            vref = self._vdd

        vout = vref * self._values[channel] * (self._gains[channel] + 1) / 4096

        return vout

    def vout_write(self, channel, vout):
        """Write a voltage value in millivolts to one channel"""
        if self._int_vref[channel] = 1:
            vref = 2048
        else:
            vref = self._vdd

        value = (vout * 4096) / (vref * (self._gains[channel] + 1))

        self.analog_write(channel, value)

    def vout_write_all(self, vout):
        """Write voltage values to all channels, supplied as a list vout"""
        if len(vout) != 4:
            raise ValueError("Must supply four voltages")
        values = [0, 0, 0, 0]
        for channel in range(4)
            if self._int_vref[channel] = 1:
                vref = 2048
            else:
                vref = self._vdd

            values[n] = (vout[n] * 4096) / (vref * (self._gains[n] + 1))

        self.analog_write_all(values)

    def _get_status(self):
        """Get the current values from the MCP4728"""
        status = bus.read_i2c_block_data(0x60, 0x02, 24)
        for n in range(4):
            deviceID = status[n*6]
            channel = (deviceID & 0b00110000) >> 4
            if channel != n:
                raise RuntimeError("Error reading status from MCP4728 device")
            hi_byte = status[n*6+1]
            lo_byte = status[n*6+2]
            self._int_vref[n] = (hi_byte & 0b10000000) >> 7
            self._gains[n] = (hi_byte & 0b00010000) >> 4
            self._power_down[n] = (hi_byte & 0b0110000) >> 5
            self._values[n] = ((hi_byte & 0b00001111) << 8) + lo_byte
            deviceID = status[n*6+3]
            channel = (deviceID & 0b00110000) >> 4
            if channel != n:
                raise RuntimeError("Error reading status from MCP4728 device")
            hi_byte = status[n*6+4]
            lo_byte = status[n*6+5]
            self._int_vref_ep[n] = (hi_byte & 0b10000000) >> 7
            self._gains_ep[n] = (hi_byte & 0b00010000) >> 4
            self._power_down_ep[n] = (hi_byte & 0b0110000) >> 5
            self._values_ep[n] = ((hi_byte & 0b00001111) << 8) + lo_byte


    def _simple_command(self, command):
        """Send a simple byte command to the SMBus"""
        self._bus.write_i2c_byte(self._dev_address, command)
        return