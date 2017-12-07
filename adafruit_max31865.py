# The MIT License (MIT)
#
# Copyright (c) 2017 Tony DiCola for Adafruit Industries
#
# Permission is hereby granted, free of charge, to any person obtaining a copy
# of this software and associated documentation files (the "Software"), to deal
# in the Software without restriction, including without limitation the rights
# to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
# copies of the Software, and to permit persons to whom the Software is
# furnished to do so, subject to the following conditions:
#
# The above copyright notice and this permission notice shall be included in
# all copies or substantial portions of the Software.
#
# THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
# IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
# FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
# AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
# LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
# OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN
# THE SOFTWARE.
"""
`adafruit_max31865`
====================================================

CircuitPython module for the MAX31865 platinum RTD temperature sensor.  See
examples/simpletest.py for an example of the usage.

* Author(s): Tony DiCola
"""
import math
import time

import adafruit_bus_device.spi_device as spi_device


# Register and other constant values:
_MAX31865_CONFIG_REG            = const(0x00)
_MAX31865_CONFIG_BIAS           = const(0x80)
_MAX31865_CONFIG_MODEAUTO       = const(0x40)
_MAX31865_CONFIG_MODEOFF        = const(0x00)
_MAX31865_CONFIG_1SHOT          = const(0x20)
_MAX31865_CONFIG_3WIRE          = const(0x10)
_MAX31865_CONFIG_24WIRE         = const(0x00)
_MAX31865_CONFIG_FAULTSTAT      = const(0x02)
_MAX31865_CONFIG_FILT50HZ       = const(0x01)
_MAX31865_CONFIG_FILT60HZ       = const(0x00)
_MAX31865_RTDMSB_REG            = const(0x01)
_MAX31865_RTDLSB_REG            = const(0x02)
_MAX31865_HFAULTMSB_REG         = const(0x03)
_MAX31865_HFAULTLSB_REG         = const(0x04)
_MAX31865_LFAULTMSB_REG         = const(0x05)
_MAX31865_LFAULTLSB_REG         = const(0x06)
_MAX31865_FAULTSTAT_REG         = const(0x07)
_MAX31865_FAULT_HIGHTHRESH      = const(0x80)
_MAX31865_FAULT_LOWTHRESH       = const(0x40)
_MAX31865_FAULT_REFINLOW        = const(0x20)
_MAX31865_FAULT_REFINHIGH       = const(0x10)
_MAX31865_FAULT_RTDINLOW        = const(0x08)
_MAX31865_FAULT_OVUV            = const(0x04)
_RTD_A  = 3.9083e-3
_RTD_B  = -5.775e-7


class MAX31865:

    # Class-level buffer for reading and writing data with the sensor.
    # This reduces memory allocations but means the code is not re-entrant or
    # thread safe!
    _BUFFER = bytearray(3)

    def __init__(self, spi, cs, rtd_nominal=100, ref_resistor=430.0, wires=2):
        self.rtd_nominal = rtd_nominal
        self.ref_resistor = ref_resistor
        self._device = spi_device.SPIDevice(spi, cs, baudrate=500000,
                                            polarity=0, phase=1)
        # Set wire config register based on the number of wires specified.
        if wires not in (2, 3, 4):
            raise ValueError('Wires must be a value of 2, 3, or 4!')
        t = self._read_u8(_MAX31865_CONFIG_REG)
        if wires == 3:
            t |= _MAX31865_CONFIG_3WIRE
        else:
            # 2 or 4 wire
            t &= ~_MAX31865_CONFIG_3WIRE
        self._write_u8(_MAX31865_CONFIG_REG, t)
        # Default to no bias and no auto conversion.
        self.bias = False
        self.auto_convert = False

    def _read_u8(self, address):
        # Read an 8-bit unsigned value from the specified 8-bit address.
        with self._device as device:
            self._BUFFER[0] = address & 0x7F
            device.write(self._BUFFER, end=1)
            device.readinto(self._BUFFER, end=1)
        return self._BUFFER[0]

    def _read_u16(self, address):
        # Read a 16-bit BE unsigned value from the specified 8-bit address.
        with self._device as device:
            self._BUFFER[0] = address & 0x7F
            device.write(self._BUFFER, end=1)
            device.readinto(self._BUFFER, end=2)
        return (self._BUFFER[0] << 8) | self._BUFFER[1]

    def _write_u8(self, address, val):
        # Write an 8-bit unsigned value to the specified 8-bit address.
        with self._device as device:
            self._BUFFER[0] = (address | 0x80) & 0xFF
            self._BUFFER[1] = val & 0xFF
            device.write(self._BUFFER, end=2)

    @property
    def bias(self):
        """Get and set the boolean state of the sensor's bias (True/False)."""
        return bool(self._read_u8(_MAX31865_CONFIG_REG) & _MAX31865_CONFIG_BIAS)

    @bias.setter
    def bias(self, val):
        t = self._read_u8(_MAX31865_CONFIG_REG)
        if val:
            t |= _MAX31865_CONFIG_BIAS  # Enable bias.
        else:
            t &= ~_MAX31865_CONFIG_BIAS  # Disable bias.
        self._write_u8(_MAX31865_CONFIG_REG, t)

    @property
    def auto_convert(self):
        """Get and set the boolean state of the sensor's automatic conversion
        mode (True/False).
        """
        return bool(self._read_u8(_MAX31865_CONFIG_REG) & _MAX31865_CONFIG_MODEAUTO)

    @auto_convert.setter
    def auto_convert(self, val):
        t = self._read_u8(_MAX31865_CONFIG_REG)
        if val:
            t |= _MAX31865_CONFIG_MODEAUTO   # Enable auto convert.
        else:
            t &= ~_MAX31865_CONFIG_MODEAUTO  # Disable auto convert.
        self._write_u8(_MAX31865_CONFIG_REG, t)

    @property
    def fault(self):
        """Get the fault state of the sensor.  Use the clear_faults function
        to clear the fault state.  Returns a 6-tuple of boolean values which
        indicate if any faults are present:
          - HIGHTHRESH
          - LOWTHRESH
          - REFINLOW
          - REFINHIGH
          - RTDINLOW
          - OVUV
        """
        faults = self._read_u8(_MAX31865_FAULTSTAT_REG)
        highthresh = bool(faults & _MAX31865_FAULT_HIGHTHRESH)
        lowthresh  = bool(faults & _MAX31865_FAULT_LOWTHRESH)
        refinlow   = bool(faults & _MAX31865_FAULT_REFINLOW)
        refinhigh  = bool(faults & _MAX31865_FAULT_REFINHIGH)
        rtdinlow   = bool(faults & _MAX31865_FAULT_RTDINLOW)
        ovuv       = bool(faults & _MAX31865_FAULT_OVUV)
        return (highthresh, lowthresh, refinlow, refinhigh, rtdinlow, ovuv)

    def clear_faults(self):
        """Clear any fault state previously detected by the sensor."""
        t = self._read_u8(_MAX31865_CONFIG_REG)
        t &= ~0x2C
        t |= _MAX31865_CONFIG_FAULTSTAT
        self._write_u8(_MAX31865_CONFIG_REG, t)

    def read_rtd(self):
        """Perform a raw reading of the thermocouple and return its 15-bit
        value.  You'll need to manually convert this to temperature using the
        nominal value of the RTD and some math.  If you just want temperature
        use the temperature property instead.
        """
        self.clear_faults()
        self.bias = True
        time.sleep(0.01)
        t = self._read_u8(_MAX31865_CONFIG_REG)
        t |= _MAX31865_CONFIG_1SHOT
        self._write_u8(_MAX31865_CONFIG_REG, t)
        time.sleep(0.065)
        rtd = self._read_u16(_MAX31865_RTDMSB_REG)
        # Remove fault bit.
        rtd >>= 1
        return rtd

    @property
    def temperature(self):
        """Read the temperature of the sensor and return its value in degrees
        Celsius.
        """
        Rt = self.read_rtd()
        Rt /= 32768
        Rt *= self.ref_resistor
        Z1 = -_RTD_A
        Z2 = _RTD_A * _RTD_A - (4 * _RTD_B)
        Z3 = (4 * _RTD_B) / self.rtd_nominal
        Z4 = 2 * _RTD_B
        temp = Z2 + (Z3 * Rt)
        temp = (math.sqrt(temp) + Z1) / Z4
        if temp >= 0:
            return temp
        rpoly = Rt
        temp = -242.02
        temp += 2.2228 * rpoly
        rpoly *= Rt  # square
        temp += 2.5859e-3 * rpoly
        rpoly *= Rt # ^3
        temp -= 4.8260e-6 * rpoly
        rpoly *= Rt # ^4
        temp -= 2.8183e-8 * rpoly
        rpoly *= Rt # ^5
        temp += 1.5243e-10 * rpoly
        return temp
