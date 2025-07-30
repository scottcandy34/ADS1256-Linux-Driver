from .interface import Interface
from .constants import *

class ADS1256:
    """
    drdy: set GPIO pin number
    cs: set GPIO pin number or set to 'None' if set always on
    bus: spi bus number
    """
    def __init__(self, drdy, cs, bus):
        self.com = Interface(drdy, cs, bus)

    def init(self, maxSpeedHz, gain = ADCON.PGA.g_1, drate = DRATE.sps_30000):
        """
        maxSpeedHz: set max speed of spi device
        gain: set ads1256 gain 1 to 64
        drate: set ads1256 data rate 2.5sps to 30,000sps
        """
        self.com.init(maxSpeedHz)
        self.reset()

        id = self.readChipID()
        if id != 3 :
            print("ID Read failed   ")
            return -1

        self.writeReg(REG.status, STATUS.id | STATUS.bufen_on)
        self.writeReg(REG.mux, MUX.reset)
        self.writeReg(REG.adcon, ADCON.PGA.g_64)
        self.writeReg(REG.drate, DRATE.sps_1000)

        self.writeCMD(CMD.selfcal)
        self.com.delay_ms(40)

    def readChipID(self) -> int:
        """Read Chip ID"""
        self.com.waitforDRDY()
        id = self.readReg(REG.status)
        id = id[0] >> 4
        return id

    def reset(self):
        """Reset all registers on ads1256"""
        self.com.csLow()
        self.com.spi_writebyte([CMD.reset])
        self.com.spi_writebyte([CMD.sdatac])
        self.com.csHigh()

    def readReg(self, reg) -> [bytes, ...]:
        """Read registry value"""
        self.com.csLow()
        self.com.spi_writebyte([CMD.rreg | reg, REG.secdCMD])
        data = self.com.spi_readbytes(CMD.rdata)
        self.com.csHigh()
        return data

    def writeReg(self, reg, value):
        """Write registry value"""
        if value != self.readReg(reg)[0]:
            self.com.waitforDRDY()
            self.com.csLow()
            self.com.spi_writebyte([CMD.wreg | reg, REG.secdCMD, value])
            self.com.csHigh()
            if value != self.readReg(reg)[0]:
                print(f"Write to Register {hex(reg)} failed!")

    def writeCMD(self, cmd):
        """Send command to ads1256"""
        self.com.waitforDRDY()
        self.com.csLow()
        self.com.spi_writebyte([cmd])
        self.com.csHigh()

    def read_channels(self) -> tuple[float, float, float, float, float, float, float, float]:
        """
        Read individual channels from ads1256

        AINS: 0, 1, 2, 3, 4, 5, 6, 7
        """
        adc_val0 = self.read_digital_out(MUX.PSEL.i_0 | MUX.ncom)
        adc_val1 = self.read_digital_out(MUX.PSEL.i_1 | MUX.ncom)
        adc_val2 = self.read_digital_out(MUX.PSEL.i_2 | MUX.ncom)
        adc_val3 = self.read_digital_out(MUX.PSEL.i_3 | MUX.ncom)
        adc_val4 = self.read_digital_out(MUX.PSEL.i_4 | MUX.ncom)
        adc_val5 = self.read_digital_out(MUX.PSEL.i_5 | MUX.ncom)
        adc_val6 = self.read_digital_out(MUX.PSEL.i_6 | MUX.ncom)
        adc_val7 = self.read_digital_out(MUX.PSEL.i_7 | MUX.ncom)
        
        return (adc_val0, adc_val1, adc_val2, adc_val3, adc_val4, adc_val5, adc_val6, adc_val7)

    def read_diff_channels(self) -> tuple[float, float, float, float]:
        """
        Read differental input pairs from ads1256
        
        AIN pairs: 0-1, 2-3, 4-5, 6-7
        """
        adc_val1 = self.read_digital_out(MUX.PSEL.i_0 | MUX.NSEL.i_1)
        adc_val2 = self.read_digital_out(MUX.PSEL.i_2 | MUX.NSEL.i_3)
        adc_val3 = self.read_digital_out(MUX.PSEL.i_4 | MUX.NSEL.i_5)
        adc_val4 = self.read_digital_out(MUX.PSEL.i_6 | MUX.NSEL.i_7)

        return (adc_val1, adc_val2, adc_val3, adc_val4)

    def read_digital_out(self, mux) -> float:
        """Get the digital output from the conversion"""
        self.com.waitforDRDY()
        self.com.csLow()
        self.com.spi_writebyte([CMD.wreg | REG.mux,  REG.secdCMD, mux])
        self.com.spi_writebyte([CMD.sync])
        self.com.spi_writebyte([CMD.wakeup])
        self.com.spi_writebyte([CMD.rdata])
        
        buf = self.com.spi_readbytes(3)
        self.com.csHigh()

        read = (buf[0]<<16) & 0xff0000 # MSB
        read |= (buf[1]<<8) & 0xff00 # Mid-byte
        read |= (buf[2]) & 0xff # LSB
        return read

    def close(self):
        """Stop communications with ADS1256 board"""
        self.com.close()

