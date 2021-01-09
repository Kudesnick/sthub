#!/usr/bin/env python

import time
import sys
import libscrc
import spidev

buf_len = 32

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

def send_msg(iface, msg_len):

    d_out = [i for i in range(0, msg_len)]
    d_out[0] = 0x01 | (iface << 4)
    d_out[1] = msg_len - 2
    d_out[-1] = libscrc.crc8(bytes(d_out[:-1]))
    res_out = ''.join('%02x '%i for i in d_out)

    d_in  = spi.xfer(d_out)
    res_in  = ''.join('%02x '%i for i in d_in )

    print ("out > ", res_out)
    print ("in  < ", res_in)

for i in range(0, 10):
    send_msg(i, buf_len)

spi.close()
sys.exit(0)
