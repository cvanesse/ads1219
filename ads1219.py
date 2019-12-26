# Raspberry Pi driver for the Texas Instruments ADS1219 ADC

import struct, time

_CHANNEL_MASK = 0b11100000
_GAIN_MASK = 0b00010000
_DR_MASK = 0b00001100
_CM_MASK = 0b00000010
_VREF_MASK = 0b00000001

_COMMAND_RESET = 0b00000110
_COMMAND_START_SYNC = 0b00001000
_COMMAND_POWERDOWN = 0b00000010
_COMMAND_RDATA = 0b00010000
_COMMAND_RREG_CONFIG = 0b00100000
_COMMAND_RREG_STATUS = 0b00100100
_COMMAND_WREG_CONFIG = 0b01000000

_DRDY_MASK = 0b10000000
_DRDY_NO_NEW_RESULT = 0b00000000    # No new conversion result available
_DRDY_NEW_RESULT_READY = 0b10000000 # New conversion result ready

class ADS1219:
    CHANNEL_AIN0_AIN1 = 0b00000000  # Differential P = AIN0, N = AIN1 (default)
    CHANNEL_AIN2_AIN3 = 0b00100000  # Differential P = AIN2, N = AIN3
    CHANNEL_AIN1_AIN2 = 0b01000000  # Differential P = AIN1, N = AIN
    CHANNEL_AIN0 = 0b01100000   # Single-ended AIN0
    CHANNEL_AIN1 = 0b10000000       # Single-ended AIN1
    CHANNEL_AIN2 = 0b10100000       # Single-ended AIN2
    CHANNEL_AIN3 = 0b11000000       # Single-ended AIN3
    CHANNEL_MID_AVDD = 0b11100000   # Mid-supply   P = AVDD/2, N = AVDD/2
    
    GAIN_1X = 0b00000 # Gain = 1 (default)
    GAIN_4X = 0b10000 # Gain = 4
    
    DR_20_SPS = 0b0000   # Data rate = 20 SPS (default)
    DR_90_SPS = 0b0100   # Data rate = 90 SPS
    DR_330_SPS = 0b1000  # Data rate = 330 SPS
    DR_1000_SPS = 0b1100 # Data rate = 1000 SPS

    CM_SINGLE = 0b00     # Single-shot conversion mode (default)
    CM_CONTINUOUS = 0b10 # Continuous conversion mode

    VREF_INTERNAL = 0b0 # Internal 2.048V reference (default)
    VREF_EXTERNAL = 0b1 # External reference
    
    VREF_INTERNAL_MV = 2048 # Internal reference voltage = 2048 mV
    POSITIVE_CODE_RANGE = 0x7FFFFF # 23 bits of positive range    

    def __init__(self, i2c, address=0x40):
        self._i2c = i2c
        self._address = address
        self.reset()
                
    def _read_modify_write_config(self, mask, value):
        as_is = self.read_config()
        to_be = (as_is & ~mask) | value 
        wreg = struct.pack('BB', _COMMAND_WREG_CONFIG, to_be)
        self._i2c.write_i2c_block_data(self._address, 0, wreg)
        
    def read_config(self):
        rreg = struct.pack('B', _COMMAND_RREG_CONFIG) 
        self._i2c.write_i2c_block_data(self._address, 0, rreg)
        config = self._i2c.read_i2c_block_data(self._address, 0, 1)
        return config[0]
    
    def read_status(self):
        rreg = struct.pack('B', _COMMAND_RREG_STATUS) 
        self._i2c.write_i2c_block_data(self._address, 0, rreg)
        status = self._i2c.read_i2c_block_data(self._address, 0, 1)
        return status[0]

    def set_channel(self, channel):
        self._read_modify_write_config(_CHANNEL_MASK, channel)
        
    def set_gain(self, gain):
        self._read_modify_write_config(_GAIN_MASK, gain)
        
    def set_data_rate(self, dr):
        self._read_modify_write_config(_DR_MASK, dr)
        
    def set_conversion_mode(self, cm):
        self._read_modify_write_config(_CM_MASK, cm)
        
    def set_vref(self, vref):
        self._read_modify_write_config(_VREF_MASK, vref)
        
    def read_data(self):
        if ((self.read_config() & _CM_MASK) == CM_SINGLE):
            self.start_sync()
            # loop until conversion is completed
            while((self.read_status() & _DRDY_MASK) == _DRDY_NO_NEW_RESULT):
                time.sleep(100e-6)
            
        rreg = struct.pack('B', _COMMAND_RDATA) 
        self._i2c.write_i2c_block_data(self._address, 0, rreg)
        data = self._i2c.read_i2c_block_data(self._address, 0, 3)
        return struct.unpack('>I', b'\x00' + data)[0]
    
    def read_data_irq(self):
        rreg = struct.pack('B', _COMMAND_RDATA) 
        self._i2c.write_i2c_block_data(self._address, 0, rreg)
        data = self._i2c.read_i2c_block_data(self._address, 0, 3)
        return struct.unpack('>I', b'\x00' + data)[0]
        
    def reset(self):
        data = struct.pack('B', _COMMAND_RESET)
        self._i2c.write_i2c_block_data(self._address, 0, data)

    def start_sync(self):
        data = struct.pack('B', _COMMAND_START_SYNC)
        self._i2c.write_i2c_block_data(self._address, 0, data)

    def powerdown(self):
        data = struct.pack('B', _COMMAND_POWERDOWN)
        self._i2c.write_i2c_block_data(self._address, 0, data)