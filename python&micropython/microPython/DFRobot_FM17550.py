import time
from machine import Pin, I2C



MAXRLEN       = 18
MIN_STRENGTH  = 228

DEF_FIFO_LENGTH      =       64          


PCD_IDLE            =      0x00          
PCD_AUTHENT         =      0x0E          
PCD_RECEIVE         =      0x08          
PCD_TRANSMIT        =      0x04          
PCD_TRANSCEIVE      =      0x0C          
PCD_RESETPHASE      =      0x0F          
PCD_CALCCRC         =      0x03          

PICC_REQIDL         =      0x26         
PICC_REQALL          =     0x52          
PICC_ANTICOLL1      =      0x93          
PICC_ANTICOLL2      =      0x95          
PICC_AUTHENT1A      =      0x60          
PICC_AUTHENT1B      =      0x61          
PICC_READ           =      0x30          
PICC_WRITE          =      0xA0          
PICC_DECREMENT      =      0xC0          
PICC_INCREMENT       =     0xC1          
PICC_RESTORE        =      0xC2          
PICC_TRANSFER       =      0xB0          
PICC_HALT           =      0x50          

RFU00            =     0x00    
CommandReg       =     0x01    
ComIEnReg        =     0x02    
DivlEnReg        =     0x03    
ComIrqReg        =     0x04    
DivIrqReg        =     0x05
ErrorReg         =     0x06    
Status1Reg       =     0x07    
Status2Reg       =     0x08    
FIFODataReg      =     0x09
FIFOLevelReg      =    0x0A
WaterLevelReg     =    0x0B
ControlReg       =     0x0C
BitFramingReg    =     0x0D
CollReg          =     0x0E
RFU0F             =    0x0F

RFU10             =    0x10
ModeReg          =     0x11
TxModeReg        =     0x12
RxModeReg        =     0x13
TxControlReg     =     0x14
TxASKReg         =     0x15
TxSelReg         =     0x16
RxSelReg         =     0x17
RxThresholdReg   =     0x18
DemodReg         =     0x19
RFU1A            =     0x1A
RFU1B            =     0x1B
MifareReg        =     0x1C
RFU1D            =     0x1D
RFU1E             =    0x1E
SerialSpeedReg   =     0x1F

RFU20            =     0x20  
CRCResultRegM    =     0x21
CRCResultRegL    =     0x22
RFU23            =     0x23
ModWidthReg      =     0x24
RFU25            =     0x25
RFCfgReg         =     0x26
GsNReg           =     0x27
CWGsCfgReg       =     0x28
ModGsCfgReg      =     0x29
TModeReg         =     0x2A
TPrescalerReg    =     0x2B
TReloadRegH      =     0x2C
TReloadRegL      =     0x2D
TCounterValueRegH  =   0x2E
TCounterValueRegL  =   0x2F

RFU30           =      0x30
TestSel1Reg      =     0x31
TestSel2Reg      =     0x32
TestPinEnReg     =     0x33
TestPinValueReg  =     0x34
TestBusReg       =     0x35
AutoTestReg      =     0x36
VersionReg       =     0x37
AnalogTestReg    =     0x38
TestDAC1Reg      =     0x39  
TestDAC2Reg      =     0x3A   
TestADCReg       =     0x3B   
RFU3C            =     0x3C   
RFU3D            =     0x3D   
RFU3E            =     0x3E   
RFU3F		     =     0x3F

MI_OK            =              0 
MI_CHK_OK        =              0 
MI_CRC_ZERO      =              0 

MI_CRC_NOTZERO    =             1 

MI_NOTAGERR       =          0xFF 
MI_CHK_FAILED     =          0xFF 
MI_CRCERR         =          0xFE 
MI_CHK_COMPERR    =          0xFE 
MI_EMPTY          =          0xFD 
MI_AUTHERR        =          0xFC 
MI_PARITYERR      =          0xFB 
MI_CODEERR        =          0xFA 

MI_SERNRERR       =          0xF8 
MI_KEYERR         =          0xF7 
MI_NOTAUTHERR     =          0xF6 
MI_BITCOUNTERR    =          0xF5 
MI_BYTECOUNTERR   =          0xF4 
MI_IDLE           =          0xF3 
MI_TRANSERR       =          0xF2 
MI_WRITEERR       =          0xF1 
MI_INCRERR        =          0xF0 
MI_DECRERR        =          0xEF 
MI_READERR        =          0xEE 
MI_OVFLERR        =          0xED 
MI_POLLING        =          0xEC 
MI_FRAMINGERR     =          0xEB 
MI_ACCESSERR      =          0xEA 
MI_UNKNOWN_COMMAND  =        0xE9 
MI_COLLERR          =        0xE8 
MI_RESETERR         =        0xE7 
MI_INITERR          =        0xE7 
MI_INTERFACEERR     =        0xE7 
MI_ACCESSTIMEOUT    =        0xE5 
MI_NOBITWISEANTICOLL  =      0xE4 
MI_QUIT               =      0xE2 

MI_RECBUF_OVERFLOW    =      0xCF 
MI_SENDBYTENR         =      0xCE 

MI_SENDBUF_OVERFLOW     =    0xCC 
MI_BAUDRATE_NOT_SUPPORTED =  0xCB 
MI_SAME_BAUDRATE_REQUIRED  = 0xCA 

MI_WRONG_PARAMETER_VALUE  =  0xC5 

MI_BREAK                  =  0x9E 
MI_NY_IMPLEMENTED         =  0x9D 
MI_NO_MFRC                =  0x9C 
MI_MFRC_NOTAUTH           =  0x9B 
MI_WRONG_DES_MODE         =  0x9A 
MI_HOST_AUTH_FAILED       =  0x99 

MI_WRONG_LOAD_MODE        =  0x97 
MI_WRONG_DESKEY           =  0x96 
MI_MKLOAD_FAILED          =  0x95 
MI_FIFOERR                =  0x94 
MI_WRONG_ADDR             =  0x93 
MI_DESKEYLOAD_FAILED      =  0x92 

MI_WRONG_SEL_CNT          =  0x8F 
MI_RC531_WRONG_READVALUE  =  0x8E 
MI_WRONG_TEST_MODE        =  0x8C 
MI_TEST_FAILED            =  0x8B 
MI_TOC_ERROR              =  0x8A 
MI_COMM_ABORT             =  0x89 
MI_INVALID_BASE           =  0x88 
MI_MFRC_RESET             =  0x87 
MI_WRONG_VALUE            =  0x86 
MI_VALERR                 =  0x85

_STATUS_OK = 0x00

class Rfid(object):
    password_buffer=[0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF]
    RC552_I2C_ADDR = 0x28
    uuid=[0]
    def __init__(self,scl=48,sda=47,bus=0):
        """
        @brief Module I2C communication init
        @param i2c_addr - I2C communication address
        @param scl - I2C scl pin
        @param sda - I2C sda pin
        @param bus - I2C bus
        """
        self._addr = self.RC552_I2C_ADDR
        self._i2c  = I2C(bus,scl=Pin(scl),sda=Pin(sda),freq=400000)
        self.block_data = [0]*16

    def read_raw_rc(self,address):
        """
        @brief reads 1 byte of data from the specified address
        @param address: register address
        @return Read data
        """
        try:
            self._i2c.writeto(self._addr, bytes([address])) 
            read_data = self._i2c.readfrom(self._addr, 1)   

            return read_data[0] if read_data else 0         
        except Exception as e:
            return 0
        
    def write_raw_rc(self,address, value):
        """
        @brief writes 1 byte of data to the specified address
        @param address: register address
        @param value: data to be written
        """
        try:
            self._i2c.writeto(self._addr, bytes([address, value])) 
        except Exception as e:
            pass
    
    def set_bit_mask(self, reg, mask):
        """
        @brief sets the bits in the register
        @param reg: register address
        @param mask: indicates the mask of the bit to be set
        """
        try:
            tmp = self.read_raw_rc(reg)
            tmp |= mask
            self.write_raw_rc(reg, tmp)
        except Exception as e:
            print(f"Error in set_bit_mask: {e}")

    def clear_bit_mask(self, reg, mask):
        """
        @brief clears bits in the register
        @param reg: register address
        @param mask: indicates the mask of the bit to be cleared
        """
        try:
            tmp = self.read_raw_rc(reg)
            tmp &= ~mask
            self.write_raw_rc(reg, tmp)
        except Exception as e:
            print(f"Error in clear_bit_mask: {e}")

    def PcdReset(self):
        """
        @brief is used to initialize the reset and configure the module.
        @return: 0
        """
        self.write_raw_rc(CommandReg, PCD_RESETPHASE)
        time.sleep_ms(100)
        self.write_raw_rc(ModeReg, 0x3D)
        self.write_raw_rc(TReloadRegL, 30)
        self.write_raw_rc(TReloadRegH, 0)
        self.write_raw_rc(TModeReg, 0x8D)
        self.write_raw_rc(TPrescalerReg, 0x3E)
        return MI_OK            
    
    def pcd_com(self, command, p_in_data, in_len_byte, p_out_data, p_out_len_bit):
        """
        @brief command control
        @param command: instruction type (PCD_AUTHENT or PCD_TRANSCEIVE)
        @param p_in_data: Input data array
        @param in_len_byte: indicates the length of the data
        @param p_out_data: Output data array (as outgoing argument)
        @param p_out_len_bit: Length of output data (in bits, as outgoing parameter)
        @return: status code (such as MI_OK or MI_ERR)
        """
        MI_OK = 0
        MI_ERR = -1
        MI_NOTAGERR = -2
        MAXRLEN = 16
        irq_en = 0x00
        wait_for = 0x00
        if command == PCD_AUTHENT:
            irq_en = 0x12
            wait_for = 0x10
        elif command == PCD_TRANSCEIVE:
            irq_en = 0x77
            wait_for = 0x30
        self.write_raw_rc(ComIEnReg, irq_en | 0x80) 
        self.clear_bit_mask(ComIrqReg, 0x80)        
        self.write_raw_rc(CommandReg, PCD_IDLE)     
        self.set_bit_mask(FIFOLevelReg, 0x80)       
        for i in range(in_len_byte):
            self.write_raw_rc(FIFODataReg, p_in_data[i])
        self.write_raw_rc(CommandReg, command)
        if command == PCD_TRANSCEIVE:
            self.set_bit_mask(BitFramingReg, 0x80)
        timeout = 6000
        while timeout:
            n = self.read_raw_rc(ComIrqReg)
            if n & 0x01 or n & wait_for: 
                break
            timeout -= 1
        self.clear_bit_mask(BitFramingReg, 0x80)
        status = MI_ERR
        if timeout:
            if not (self.read_raw_rc(ErrorReg) & 0x1B):
                status = MI_OK
                if n & irq_en & 0x01:
                    status = MI_NOTAGERR
                if command == PCD_TRANSCEIVE:
                    n = self.read_raw_rc(FIFOLevelReg)
                    last_bits = self.read_raw_rc(ControlReg) & 0x07
                    if last_bits:
                        p_out_len_bit[0] = (n - 1) * 8 + last_bits
                    else:
                        p_out_len_bit[0] = n * 8
                    if n == 0:
                        n = 1
                    if n > MAXRLEN:
                        n = MAXRLEN
                    p_out_data.clear() 
                    for i in range(n):
                        p_out_data.append(self.read_raw_rc(FIFODataReg))
            else:
                status = MI_ERR
        self.set_bit_mask(ControlReg, 0x80)
        self.write_raw_rc(CommandReg, PCD_IDLE)
        
        return status
    
    def pcd_request(self, req_code):
        """
        @brief implements the PcdRequest function and sends a request command to get the card type.
        @param req_code: Request code (REQA or WUPA)
        @return: (status code, p_tag_type), the status code is MI_OK or MI_ERR, and p_tag_type is the card type list
        """
        MI_OK = 0
        MI_ERR = -1
        p_tag_type = [0, 0]

        self.clear_bit_mask(Status2Reg, 0x08)

        self.write_raw_rc(BitFramingReg, 0x07)
        self.set_bit_mask(TxControlReg, 0x03)

        uc_com_mf522_buf = [req_code]

        p_out_data = []
        p_out_len_bit = [0]
        status = self.pcd_com(PCD_TRANSCEIVE, uc_com_mf522_buf, 1, p_out_data, p_out_len_bit)

        if status == MI_OK and p_out_len_bit[0] == 0x10:
            p_tag_type[0] = p_out_data[0]
            p_tag_type[1] = p_out_data[1]
        else:
            status = MI_ERR
    
        return status, p_tag_type

    def pcd_anticoll(self):
        """
        @brief implements the PcdAnticoll function to obtain the serial number of the card.
        @return: List of card serial numbers (length 4, return list of all zeros if unsuccessful read)
        """
        MI_OK = 0
        PICC_ANTICOLL1 = 0x93 
        MAXRLEN = 16          
        
        p_snr = [0] * 4

        self.clear_bit_mask(Status2Reg, 0x08)
        self.write_raw_rc(BitFramingReg, 0x00)
        self.clear_bit_mask(CollReg, 0x80)
    
        uc_com_mf522_buf = [PICC_ANTICOLL1, 0x20]
    
        p_out_data = []
        p_out_len_bit = [0]
        status = self.pcd_com(PCD_TRANSCEIVE, uc_com_mf522_buf, 2, p_out_data, p_out_len_bit)

        if status == MI_OK and len(p_out_data) >= 5:
            snr_check = 0
            for i in range(4):
                p_snr[i] = p_out_data[i]
                snr_check ^= p_out_data[i]

            if snr_check != p_out_data[4]:
                p_snr = [0] * 4  
    
        self.set_bit_mask(CollReg, 0x80)
    
        return p_snr
    
    def pcd_select(self, p_snr):
        """
        @brief implements the PcdSelect function, which is used to select specific RFID cards.
        @param p_snr: Input parameter, serial number of the card (4 bytes).
        @return: indicates the status code, such as MI_OK or MI_ERR.
        """
        MI_OK = 0
        MI_ERR = -1
        PICC_ANTICOLL1 = 0x93
        MAXRLEN = 18
    
        uc_com_mf522_buf = [0] * MAXRLEN
        uc_com_mf522_buf[0] = PICC_ANTICOLL1
        uc_com_mf522_buf[1] = 0x70
        uc_com_mf522_buf[6] = 0
        
        
        crc = [0] * 2
    
        for i in range(4):
            uc_com_mf522_buf[i + 2] = p_snr[i]
            uc_com_mf522_buf[6] ^= p_snr[i]
    
        self.calculate_crc(uc_com_mf522_buf, 7, crc)
    
        self.clear_bit_mask(Status2Reg, 0x08)
        uc_com_mf522_buf[7]=crc[0]
        uc_com_mf522_buf[8]=crc[1]
        p_out_data = []
        p_out_len_bit = [0]
        status = self.pcd_com(PCD_TRANSCEIVE, uc_com_mf522_buf, 9, p_out_data, p_out_len_bit)
    
        if status == MI_OK and p_out_len_bit[0] == 0x18:
            status = MI_OK
        else:
            status = MI_ERR
        return status

    def pcd_auth_state(self, auth_mode, addr, p_key, p_snr):
        """
        @brief implements the PcdAuthState function and is used to authenticate the specified block of the card.
        @param auth_mode: indicates the authentication mode, such as KEY_A or KEY_B.
        @param addr: Block address to be authenticated.
        @param p_key: indicates the key (6 bytes).
        @param p_snr: Card serial number (4 bytes).
        @return: indicates the status code, such as MI_OK or MI_ERR.
        """
        MI_OK = 0
        MI_ERR = -1
        MAXRLEN = 16
    
        uc_com_mf522_buf = [0] * MAXRLEN
        uc_com_mf522_buf[0] = auth_mode
        uc_com_mf522_buf[1] = addr
    
        for i in range(6):
            uc_com_mf522_buf[i + 2] = p_key[i]
    
        for i in range(4):
            uc_com_mf522_buf[i + 8] = p_snr[i]
    
        p_out_data = []
        p_out_len_bit = [0]
        status = self.pcd_com(PCD_AUTHENT, uc_com_mf522_buf, 12, p_out_data, p_out_len_bit)
    
        if status != MI_OK or not (self.read_raw_rc(Status2Reg) & 0x08):
            status = MI_ERR
        return status

    def pcd_read(self, addr):
        """
        @brief implements the PcdRead function and reads data from the specified block address.
        @param addr: Block address to be read.
        @return: (status code, data array).
                 The status code can be MI_OK or MI_ERR.
                 The data array is 16 bytes of read data.
        """
        MI_OK = 0
        MI_ERR = -1
        MAXRLEN = 16
    
        uc_com_mf522_buf = [0] * MAXRLEN
        uc_com_mf522_buf[0] = PICC_READ
        uc_com_mf522_buf[1] = addr
        crc = [0] * 2

        self.calculate_crc(uc_com_mf522_buf[:2],2, crc)
    
        uc_com_mf522_buf[2] = crc[0]
        uc_com_mf522_buf[3] = crc[1]
        
        p_out_data = [0] * MAXRLEN
        p_out_len_bit = [0]
        status = self.pcd_com(PCD_TRANSCEIVE, uc_com_mf522_buf, 4, p_out_data, p_out_len_bit)
    
        if status == MI_OK and p_out_len_bit[0] == 0x90:
            read_data = p_out_data[:16]
            return MI_OK, read_data
        else:
            return MI_ERR, None

    def _read_data(self, block):
        """
        @brief reads data from the specified block address and optionally decrypts it.
        @param block: block address to read.
        @return: reads the status and data (return (0, buf) on success, return (error_code, None) on failure).
        """
        MI_OK = 0
        result = self.pcd_auth_state(0x60, block, self.password_buffer, self.uuid)
        if result != MI_OK:
            return "error"
    
        result, data = self.pcd_read(block)
        if result != MI_OK:
            return "error"
        return data

    def pcd_write(self,addr, pData):
        """
        @brief Writes data to a specific card block.
        @param addr Address of the block to write.
        @param pData Pointer to an array containing the data to write.
        @return Returns 0 on success; otherwise, returns an error code.
        """
        MI_OK = 0
        MI_ERR = -1
        status = MI_OK
        unLen = [0]
        ucComMF522Buf = [0] * 18

        ucComMF522Buf[0] = PICC_WRITE
        ucComMF522Buf[1] = addr
        crc = [0] * 2

        self.calculate_crc(ucComMF522Buf, 2, crc)
        ucComMF522Buf[2] = crc[0]
        ucComMF522Buf[3] = crc[1]
        status = self.pcd_com(PCD_TRANSCEIVE, ucComMF522Buf, 4, ucComMF522Buf, unLen)
    
        if status != MI_OK or unLen[0] != 4 or (ucComMF522Buf[0] & 0x0F) != 0x0A:
            status = MI_ERR
        ucComMF522Buf1 = [0] * 18
        if status == MI_OK:
            for i in range(16):
                ucComMF522Buf1[i] = pData[i]
            self.calculate_crc(ucComMF522Buf1, 16, crc)
            ucComMF522Buf1[16] = crc[0]
            ucComMF522Buf1[17] = crc[1]
            status = self.pcd_com(PCD_TRANSCEIVE, ucComMF522Buf1, 18, ucComMF522Buf1, unLen)
            if status != MI_OK or unLen[0] != 4 or (ucComMF522Buf1[0] & 0x0F) != 0x0A:
                status = MI_ERR
        return status
    
    def _write_data(self, block, rf_buffer):
        """
        @brief writes data to the specified RFID card block.
        @param block: block address to write to.
        @param rf_buffer: The data buffer to be written (the length should be 16 bytes).
        @return: The status of the write operation (0 indicates success, non-zero indicates failure).
        """
        MI_OK = 0
        result = self.pcd_auth_state(0x60, block, self.password_buffer, self.uuid)
        if result != MI_OK:
            return result
        result = self.pcd_write(block, rf_buffer)
        return result

    def PcdHalt(self):
        """
        @brief Halts the currently connected RFID card.
        @return Returns 0 on success; otherwise, returns an error code.
        """
        status = MI_OK
        unLen = [0]
        ucComMF522Buf = [0] * MAXRLEN
        ucComMF522Buf[0] = PICC_HALT
        ucComMF522Buf[1] = 0
        self.calculate_crc(ucComMF522Buf, 2, ucComMF522Buf[2:4])
        status = self.pcd_com(PCD_TRANSCEIVE, ucComMF522Buf, 4, ucComMF522Buf, unLen)
        return status
  
    def MIF_Halt(self):
        """
        @brief Sends a halt command to a connected card.
        @return Returns 0 on success; otherwise, returns an error code.
        """
        status = MI_OK
        unLen = [0]
        ucComMF522Buf = [0] * MAXRLEN
        ucComMF522Buf[0] = PICC_HALT
        ucComMF522Buf[1] = 0
        self.calculate_crc(ucComMF522Buf, 2, ucComMF522Buf[2:4])
        status = self.pcd_com(PCD_TRANSCEIVE, ucComMF522Buf, 4, ucComMF522Buf, unLen)
        return status

    def calculate_crc(self, pIndata, length, pOutData):
        """
        @brief Calculates the CRC checksum for the given data.
        @param pIndata Pointer to the input data array.
        @param len Length of the input data array.
        @param pOutData Pointer to a buffer for storing the CRC checksum.
        """
        i = 0
        n = 0
        self.clear_bit_mask(DivIrqReg, 0x04)
        self.write_raw_rc(CommandReg, PCD_IDLE)
        self.set_bit_mask(FIFOLevelReg, 0x80)
        for i in range(length):
            self.write_raw_rc(FIFODataReg, pIndata[i])
        self.write_raw_rc(CommandReg, PCD_CALCCRC) 
        
        i = 0xFF
        while i != 0:
            n = self.read_raw_rc(DivIrqReg)
            i -= 1
            if n & 0x04:
                break
        pOutData[0] = self.read_raw_rc(CRCResultRegL)  
        pOutData[1] = self.read_raw_rc(CRCResultRegM)  
 
    def PcdAntennaOn(self):
        """
        MicroPython implementation of opening the antenna.
        """
        self.write_raw_rc(TxASKReg, 0x40)
        time.sleep_ms(10)
        i = self.read_raw_rc(TxControlReg)
        if not (i & 0x03):
            self.set_bit_mask(TxControlReg, 0x03)
        i = self.read_raw_rc(TxASKReg)     


    def begin(self):
        """!
        @brief Function begin.
        """
        self.PcdReset()
        self.PcdAntennaOn()   

    def read_data(self, block, index=None):
        """!
        @brief reads data from the specified block of the MIFARE classic NFC smart card/tag.
        @param block - The number of the block to read
        @param index - Data offset (1-16). When index is None, it indicates the size of a block to be read. When index is greater than 0, it indicates a byte to be read
        @return Indicates the read data
        """
        #print("read_data")
        if not self.scan():
            return "no card!"
        
        data = self._read_data(block)
        if (
            data == "error"
            or data == "read error!"
            or data == "read timeout!"
            or data == "wake up error!"
            or data == "false"
        ):
            return None
        self.block_data = data
        if index is None:
            
            return data
        else:
            if index>0:
                return self.block_data[index - 1]
            return None

    def write_data(self, block,data,index=0):
        """!
        @brief writes data to MIFARE classic NFC smart card/tag.
        @param block - The number of the block to be written
        @param data - Data to be written, when index is 0, indicates the size of a block to be written, when index is greater than 0, indicates a byte to be written
        @param index - Data offset (1-16)
        @return Boolean type, operation result
        """
        if not self.scan():
            return "no card!"
        
        if isinstance(data, str):
            real_val = []
            for i in data:
                real_val.append(int(ord(i)))
            if len(real_val) < 16:
                for i in range(16 - len(real_val)):
                    real_val.append(0)
            elif len(real_val) > 16:
                return False
        if isinstance(data, list):
            real_val = []
            if len(data) < 16:
                for i in range(16 - len(data)):
                    data.append(0)
            elif len(data) > 16:
                return False
            real_val = data
        index = max(min(index, 16), 1)
        if isinstance(data, int):
            #print(self.block_data)
            self.read_data(block)
            self.block_data[index - 1] = data
            self._write_data(block, self.block_data)
        else:
            block_data = [0 for i in range(index - 1)]
            block_data[index:] = real_val
            self._write_data(block, block_data)
        return True  
          
    def write_index_data(self, block, index, data):
        """!
        @brief writes data to MIFARE classic NFC smart card/tag.
        @param block - The number of the block to be written
        @param data - Data to be written, when index is 0, indicates the size of a block to be written, when index is greater than 0, indicates a byte to be written
        @param index - Data offset (1-16)
        @return Boolean type, operation result
        """
        if not self.scan():
            return "no card!"
        
        if isinstance(data, str):
            real_val = []
            for i in data:
                real_val.append(int(ord(i)))
            if len(real_val) < 16:
                for i in range(16 - len(real_val)):
                    real_val.append(0)
            elif len(real_val) > 16:
                return False
        if isinstance(data, list):
            real_val = []
            if len(data) < 16:
                for i in range(16 - len(data)):
                    data.append(0)
            elif len(data) > 16:
                return False
            real_val = data
        index = max(min(index, 16), 1)
        if isinstance(data, int):
            #print(self.block_data)
            self.read_data(block)
            self.block_data[index - 1] = data
            self._write_data(block, self.block_data)
        else:
            block_data = [0 for i in range(index - 1)]
            block_data[index:] = real_val
            self._write_data(block, block_data)
        return True    

    def read_protocol(self):
        state,self.nfc_protocol = self.pcd_request(0x52)
        if state != MI_OK: 
            return "no card!"
        if self.nfc_protocol[0] == 0x04 and self.nfc_protocol[1] == 0x00:
            return "mifare"
        elif self.nfc_protocol[0] == 0x02 and self.nfc_protocol[1] == 0x00:
            return "mifare"
        elif self.nfc_protocol[0] == 0x44 and self.nfc_protocol[1] == 0x00:
            return "MF-UltraLight"
        elif self.nfc_protocol[0] == 0x08 and self.nfc_protocol[1] == 0x00:
            return "mifare"
        elif self.nfc_protocol[0] == 0x44 and self.nfc_protocol[1] == 0x03:
            return "MF Desire"
        else:
            return "Unknown"

    def scan(self,uuid=""):
        """!
        @brief scans to determine if there is an NFC smart card/label.
        @param uuid - When the uuid is set, this method is used to scan the card of the uuid. If the UUID is not set, it detects whether the nfc card exists
        @return Boolean type, operation result
        @retval True: Find the card
        @retval False: No card
        """
        self.PcdHalt()
        state,self.nfc_protocol = self.pcd_request(0x52)
        ret = True
        if state != MI_OK: 
            self.card_type= "no card!"
            ret = False
        if self.nfc_protocol[0] == 0x04 and self.nfc_protocol[1] == 0x00:
            self.card_type = "mifare"
        elif self.nfc_protocol[0] == 0x02 and self.nfc_protocol[1] == 0x00:
            self.card_type = "mifare"
        elif self.nfc_protocol[0] == 0x44 and self.nfc_protocol[1] == 0x00:
            self.card_type = "MF-UltraLight"
        elif self.nfc_protocol[0] == 0x08 and self.nfc_protocol[1] == 0x00:
            self.card_type = "mifare"
        elif self.nfc_protocol[0] == 0x44 and self.nfc_protocol[1] == 0x03:
            self.card_type = "MF Desire"
        else:
            self.card_type = "Unknown"
            ret = False
        if ret:
            self.uuid = self.pcd_anticoll()
            self.scan_serial_num = int.from_bytes(bytes(self.uuid),'big')
            if self.pcd_select(self.uuid)!=0:
              ret = False  
        if uuid!="":
            uid="".join([str(hex(u))[2:] for u in self.uuid])
            uuid = uuid.lower()
            if uuid!=uid:
                ret = False
        return ret

    def read_uid(self):
        """!
        @brief Obtain the UID of the card.
        @return UID of the card.
        """
        if self.scan():
            return "".join(["%02x" % u for u in self.uuid])
        else:
            return "no card!"


