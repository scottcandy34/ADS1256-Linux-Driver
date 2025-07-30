class CMD:
    """
    SPI COMMAND DEFINITIONS (p34)
    SYSTEM CONTROL
    """

    wakeup = 0x00
    """
    Completes SYNC and Exits Standby Mode 0000  0000 (00h)
    """
    
    rdata = 0x01
    """
    Read Data 0000  0001 (01h)
    """

    rdatac = 0x03
    """
    Read Data Continuously 0000   0011 (03h)
    """

    sdatac = 0x0F
    """
    Stop Read Data Continuously 0000   1111 (0Fh)
    """

    rreg = 0x10
    """
    Read from REG rrr 0001 rrrr (1xh)
    """

    wreg = 0x50
    """
    Write to REG rrr 0101 rrrr (5xh)
    """

    selfcal = 0xF0
    """
    Offset and Gain Self-Calibration 1111    0000 (F0h)
    """

    selfocal = 0xF1
    """
    Offset Self-Calibration 1111    0001 (F1h)
    """

    selfgcal = 0xF2
    """
    Gain Self-Calibration 1111    0010 (F2h)
    """

    sysocal = 0xF3
    """
    System Offset Calibration 1111   0011 (F3h)
    """

    sysgcal = 0xF4
    """
    System Gain Calibration 1111    0100 (F4h)
    """

    sync = 0xFC
    """
    Synchronize the A/D Conversion 1111   1100 (FCh)
    """

    standby = 0xFD
    """
    Begin Standby Mode 1111   1101 (FDh)
    """

    reset = 0xFE
    """
    Reset to Power-Up Values 1111   1110 (FEh)
    """

class REG:
    """
    ADS1256 Register (see p30 for Register Map)
    """

    secdCMD = 0x00
    """
    This is the second command byte that is to be used along with WREG and RREG
    """

    status = 0
    """x1H"""
    mux = 1
    """01H"""
    adcon = 2
    """20H"""
    drate = 3
    """F0H"""
    io = 4
    """E0H"""
    ofc0 = 5
    """xxH"""
    ofc1 = 6
    """xxH"""
    ofc2 = 7
    """xxH"""
    fsc0 = 8
    """xxH"""
    fsc1 = 9
    """xxH"""
    fsc2 = 10
    """xxH"""

class STATUS:
    """
    STATUS - Status Control Register 0 ( see p30)

    BIT7 - BIT6 -  BIT5   -  BIT4   -  BIT3   -  BIT2   -  BIT1   -  BIT0
    ID   - ID   -  ID     -  ID     -  ORDER  -  ACAL   -  BUFEN  -  DRDY

    Bits 7 - 4 ID3, ID2, ID1, ID0 Factory Programmed Identification Bits(Read Only)

    DRDY1:0 Data Ready (Read Only) Duplicates the state of the DRDY pin
    """
    
    id = 0b00110000

    reset = 0x01
    """
    Reset STATUS Register
    """
    
    order_lsb = 0b00001000
    """
    ORDER1:0  Data Output Bit Order

    Most significant Bit first (default)
    Least significant Bit first

    Input data is always shifted in most significant byte and bit first. Output data is always shifted out most significant byte first. The ORDER bit only controls the bit order of the output data within the byte.
    """
    
    acal_on = 0b00000100
    """
    ACAL1:0 Auto Calibration

    Auto Calibration Disabled (default)
    Auto Calibration Enabled

    When Auto-Calibration is enabled, self-calibration begins at the completion of the WREG command that changes the PGA (bits 0-2 of ADCON register), DR (bits 7-0 in the DRATE register) or BUFEN (bit 1 in the STATUS register) values.
    """

    bufen_on = 0b00000010
    """
    BUFEN1:0 Analog Input Buffer Enable

    Buffer Disabled (default)
    BUffer Enabled
    """

class MUX:
    """
    MUX - Multiplexer Control Register 0 (see p31 - bring together with bitwise OR |
    BIT7  - BIT6  -  BIT5   -  BIT4   -  BIT3   -  BIT2   -  BIT1   -  BIT0
    PSEL3 - PSEL2 -  PSEL1  -  PSEL0  -  NSEL3  -  NSEL2  -  NSEL1  -  NSEL0
    """
    
    reset = 0x01
    """
    Reset MUX Register
    """

    class PSEL:
        """
        PSEL3:0 Positive input channel selection bits

        P_AIN0 is (default)
        """
        i_0 = 0b00000000
        i_1 = 0b00010000
        i_2 = 0b00100000
        i_3 = 0b00110000
        i_4 = 0b01000000
        i_5 = 0b01010000
        i_6 = 0b01100000
        i_7 = 0b01110000

    pcom = 0b10000000
    """
    (when PSEL3 = 1, PSEL2, PSEL1, PSEL0 are "don't care")
    """

    class NSEL:
        """
        NSEL3:0 Negative input channel selection bits

        N_AIN1 is (default)
        """
        i_0 = 0b0000
        i_1 = 0b0001
        i_2 = 0b0010
        i_3 = 0b0011
        i_4 = 0b0100
        i_5 = 0b0101
        i_6 = 0b0110
        i_7 = 0b0111

    ncom = 0b1000
    """ 
    (when NSEL3 = 1, NSEL2, NSEL1, NSEL0 are "don't care")
    """

class ADCON:
    """
    ADCON - A/D Control Register 0 ( see p31)
    BIT7 - BIT6   -  BIT5   -  BIT4   -  BIT3   -  BIT2   -  BIT1   -  BIT0
    0    - CLK1   -  CLK0   -  SDCS1  -  SDCS0  -  PGA2   -  PGA1   -  PAG0

    Clock Out off (default by 0)
    Clock Out Frequency = fCLKIN (default by chip)

    Sensor Detect Off (default)
    """
        
    reset = 0x20
    """
    Reset ADCON Register
    """

    clk_1 = 0b00100000
    """
    CLK2:0 D0/CLKOUT Clock Out Rate Setting

    Clock Out Frequency = fCLKIN

    When not using CLKOUT, it is recommended that it be turned off. These bits can only be reset using the RESET pin.
    """

    clk_2 = 0b01000000
    """
    CLK2:0 D0/CLKOUT Clock Out Rate Setting

    Clock Out Frequency = fCLKIN/2

    When not using CLKOUT, it is recommended that it be turned off. These bits can only be reset using the RESET pin.
    """

    clk_4 = 0b01100000
    """
    CLK2:0 D0/CLKOUT Clock Out Rate Setting

    Clock Out Frequency = fCLKIN/4

    When not using CLKOUT, it is recommended that it be turned off. These bits can only be reset using the RESET pin.
    """

    sdcs_05 = 0b00001000
    """
    SDCS2:0 Sensor Detection Current Sources

    Sensor Detect Current 0.5uA

    The Sensor Detect Current Sources can be activated to verify the integrity of an external sensor supplying a signal to the ADS1255/6. A shorted sensor produces a very small signal while an open-circuit sensor produces a very large signal.
    """

    sdcs_2 = 0b00010000
    """
    SDCS2:0 Sensor Detection Current Sources

    Sensor Detect Current 2uA

    The Sensor Detect Current Sources can be activated to verify the integrity of an external sensor supplying a signal to the ADS1255/6. A shorted sensor produces a very small signal while an open-circuit sensor produces a very large signal.
    """

    sdcs_10 = 0b00011000
    """
    SDCS2:0 Sensor Detection Current Sources

    Sensor Detect Current 10uA

    The Sensor Detect Current Sources can be activated to verify the integrity of an external sensor supplying a signal to the ADS1255/6. A shorted sensor produces a very small signal while an open-circuit sensor produces a very large signal.
    """
    
    class PGA:
        """
        PGA3:0 Programmable Gain Amplifier Setting
        """
        g_1 = 0
        """Gain of 1"""
        g_2 = 1
        """Gain of 2"""
        g_4 = 2
        """Gain of 4"""
        g_8 = 3
        """Gain of 8"""
        g_16 = 4
        """Gain of 16"""
        g_32 = 5
        """Gain of 32"""
        g_64 = 6
        """Gain of 64"""

class DRATE:
    """
    DRATE - A/D Data Rate Register 0 ( see p32)
    BIT7 - BIT6   -  BIT5   -  BIT4   -  BIT3   -  BIT2   -  BIT1   -  BIT0
    DR7  - DR6    -  DR5    -  DR4    -  DR3    -  DR2    -  DR1    -  DR0
    """
    reset = 0xF0
    """Reset DRATE Register"""

    sps_30000 = 0xF0
    """30,000 SPS, Samples per sec"""
    
    sps_15000 = 0xE0
    """15,000 SPS, Samples per sec"""
    
    sps_7500 = 0xD0
    """7,500 SPS, Samples per sec"""
    
    sps_3750 = 0xC0
    """3,750 SPS, Samples per sec"""
    
    sps_2000 = 0xB0
    """2,000 SPS, Samples per sec"""
    
    sps_1000 = 0xA1
    """1,000 SPS, Samples per sec"""

    sps_500 = 0x92
    """500 SPS, Samples per sec"""

    sps_100 = 0x82
    """100 SPS, Samples per sec"""

    sps_60 = 0x72
    """60 SPS, Samples per sec"""
    
    sps_50 = 0x63
    """50 SPS, Samples per sec"""
    
    sps_30 = 0x53
    """30 SPS, Samples per sec"""
    
    sps_25 = 0x43
    """25 SPS, Samples per sec"""
    
    sps_15 = 0x33
    """15 SPS, Samples per sec"""
    
    sps_10 = 0x20
    """10 SPS, Samples per sec"""
    
    sps_5 = 0x13
    """5 SPS, Samples per sec"""
    
    sps_2d5 = 0x03
    """2.5 SPS, Samples per sec"""

class IO:
    """
    IO - GPIO Control Register 0 (see p32)
    BIT7 - BIT6   -  BIT5   -  BIT4   -  BIT3   -  BIT2   -  BIT1   -  BIT0
    DIR3 - DIR2   -  DIR1   -  DIR0   -  DIO3   -  DIO2   -  DIO1   -  DIO0

    The states of these bits control the operation of the general-purpose digital I/O pins. The ADS1256 has 4 I/O pins: D3, D2, D1, and D0/CLKOUT. The ADS1255 has two digital I/O pins: D1 and D0/CLKOUT. When using an ADS1255, the register bits DIR3, DIR2, DIO3, and DIO2 can be read from and written to but have no effect.
    
    DIR3, DIR2, DIR1 default is Input
    DIR0 default is Output

    DIO3:0 Status of Digital I/O, Read Only
    """

    reset = 0xE0
    """
    Reset I/O Register
    """

    dir3_in = 0b10000000
    """
    DIR3 - Digital I/O Direction for Pin D3
    """

    dir3_in = 0b01000000
    """
    DIR2 - Digital I/O Direction for Pin D2
    """

    dir3_in = 0b00100000
    """
    DIR1 - Digital I/O Direction for Pin D1
    """

    dir3_in = 0b00010000
    """
    DIR0 - Digital I/O Direction for Pin D0/CLKOUT
    """
