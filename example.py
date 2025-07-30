from ads1256.board import ADS1256
from ads1256.constants import DRATE, ADCON
import time

DRDY_PIN = 17

SPI_SPEED = 2500000
GAIN = ADCON.PGA.g_1
DRATE = DRATE.sps_1000

adc = ADS1256(DRDY_PIN, None, 0)
adc.init(SPI_SPEED, GAIN, DRATE)

while True:
    values = adc.read_channels()
    print(values[4])
    time.sleep(0.5)