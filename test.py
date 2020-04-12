#!/usr/bin/env python

import time
import sys
import libscrc
import spidev

spi = spidev.SpiDev()
spi.open(0, 1)

spi.bits_per_word = 8 # other value not supported for BCM2835
spi.lsbfirst = False
# cdiv      speed # cdiv     speed #  cdiv     speed 
#    2  125.0 MHz #   64   3.9 MHz #  2048   122 kHz
#    4   62.5 MHz #  128  1953 kHz #  4096    61 kHz
#    8   31.2 MHz #  256   976 kHz #  8192  30.5 kHz
#   16   15.6 MHz #  512   488 kHz # 16384  15.2 kHz
#   32    7.8 MHz # 1024   244 kHz # 32768  7629 Hz
spi.max_speed_hz = 8000 # 63 MHz - MOSI limit, 32 MHz - MISO limit
spi.mode = 0b00 # CPOL0|CPHA0
spi.cshigh = False

d_out = [0x00, 0x00, 0x22, 0x23, 0x24, 0x12]
d_out[1] = len(d_out) + 1
d_out.append(libscrc.crc8(bytes(d_out)))
res_out = ''.join('%02x '%i for i in d_out)

d_in  = spi.xfer(d_out)
res_in  = ''.join('%02x '%i for i in d_in )

print ("out > ", res_out)
print ("in  < ", res_in)

spi.close()
sys.exit(0)
