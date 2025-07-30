import RPi.GPIO as GPIO
from spidev import SpiDev
import time

class State:
    LOW = False
    HIGH = True

class Interface():
    """
    drdy: set GPIO pin number
    cs: set GPIO pin number or set to 'None' if set always on
    bus: spi bus number
    device: cs pin number 'unused'
    """
    def __init__(self, drdy, cs, bus = 0, device = 0):
        self.CS_PIN = cs
        self.DRDY_PIN = drdy
        self._drdyState = State.HIGH

        self._bus = bus
        self._device = device

        self._SPI = SpiDev()
        self._SPI.no_cs

    def init(self, maxSpeedHz = 32000000, spiMode = 0b01):
        """
        maxSpeedHz: set max speed of spi device
        spiMode: set clock polarity and phase
        """
        GPIO.setmode(GPIO.BCM)
        GPIO.setwarnings(False)
        if self.CS_PIN is not None:
            GPIO.setup(self.CS_PIN, GPIO.OUT)
        GPIO.setup(self.DRDY_PIN, GPIO.IN, pull_up_down=GPIO.PUD_UP)
        GPIO.add_event_detect(self.DRDY_PIN, GPIO.FALLING, callback=self._waitForEdge)

        self._SPI.open(self._bus, self._device)
        self._SPI.max_speed_hz = maxSpeedHz
        self._SPI.mode = spiMode

    def _waitForEdge(self, channel):
        self._drdyState = State.LOW
    
    def waitforDRDY(self):
        """Wait for Data Ready pin"""
        while self._drdyState:
            continue

        self._drdyState = State.HIGH

    def digital_write(self, pin, value):
        """Write to GPIO output"""
        GPIO.output(pin, value)

    def digital_read(self, pin) -> bool:
        """
        Read from GPIO INPUT
        Returns HIGH=1=True or LOW=0=False
        """
        return (GPIO.input(pin) == 1)

    def delay(self, sec):
        """Delay in Seconds"""
        time.sleep(sec)

    def delay_ms(self, milisec):
        """Delay in Milliseconds"""
        time.sleep(milisec / 1000.0)

    def delay_us(self, microsec):
        """Delay in Microseconds"""
        time.sleep(microsec / 1000000.0)

    def spi_writebyte(self, data):
        """Write bytes over SPI. Must be in [] array list"""
        self._SPI.writebytes(data)
        
    def spi_readbytes(self, len) -> [bytes, ...]:
        """Read bytes from SPI. Must be in [] array list"""
        return self._SPI.readbytes(len)
    
    def spi_transfer(self, data):
        self._SPI.xfer(data)

    def csLow(self):
        """Set Chip select pin to LOW"""
        if self.CS_PIN is not None:
            self.digital_write(self.CS_PIN, GPIO.LOW)

    def csHigh(self):
        """Set Chip select pin to HIGH"""
        if self.CS_PIN is not None:
            self.digital_write(self.CS_PIN, GPIO.HIGH)

    def close(self):
        """Close out GPIO and SPI Communication"""
        GPIO.cleanup()
        self._SPI.close()