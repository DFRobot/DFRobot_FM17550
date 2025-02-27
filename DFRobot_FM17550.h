/*!
 * @file DFRobot_FM17550.h
 * @brief Define the basic structure of class DFRobot_FM17550 
 * @copyright   Copyright (c) 2024 DFRobot Co.Ltd (http://www.dfrobot.com)
 * @licence     The MIT License (MIT)
 * @author [fary](feng.yang@dfrobot.com)
 * @version  V1.0
 * @date  2025-02-08
 * @get from https://www.dfrobot.com
 * @https://github.com/cdjq/DFRobot_FM17550
 */

#ifndef DFROBOT_FM17550_H
#define DFROBOT_FM17550_H
#include <Wire.h>
#include "Arduino.h"

#define ENABLE_DBG

#ifdef ENABLE_DBG
#define DBG(...) {Serial.print("["); Serial.print(__FUNCTION__); Serial.print("(): "); Serial.print(__LINE__); Serial.print(" ] "); Serial.println(__VA_ARGS__);}
#else
#define DBG(...)
#endif


#define MAXRLEN        18  ///< Maximum buffer length (16 bytes of data + CRC)
#define MIN_STRENGTH   228  ///< Minimum signal strength to consider a valid card signal

//******************************************************************
//                    RC522 FIFO Definitions
//******************************************************************
#define DEF_FIFO_LENGTH  64  ///< RC522 FIFO buffer size is 64 bytes

//******************************************************************
//                       RC522 Command Definitions
//******************************************************************
#define PCD_IDLE        0x00  ///< Idle mode, stops the current command
#define PCD_AUTHENT     0x0E  ///< Perform authentication using a key
#define PCD_RECEIVE     0x08  ///< Receive data command
#define PCD_TRANSMIT    0x04  ///< Transmit data command
#define PCD_TRANSCEIVE  0x0C  ///< Transmit and receive data
#define PCD_RESETPHASE  0x0F  ///< Reset RC522 module
#define PCD_CALCCRC     0x03  ///< Perform CRC calculation

//******************************************************************
//                  Mifare_One Card Command Definitions
//******************************************************************
#define PICC_REQIDL     0x26  ///< Detect cards in idle state
#define PICC_REQALL     0x52  ///< Detect all cards in the field
#define PICC_ANTICOLL1  0x93  ///< Anti-collision command (level 1)
#define PICC_ANTICOLL2  0x95  ///< Anti-collision command (level 2)
#define PICC_AUTHENT1A  0x60  ///< Authenticate with key A
#define PICC_AUTHENT1B  0x61  ///< Authenticate with key B
#define PICC_READ       0x30  ///< Read block data
#define PICC_WRITE      0xA0  ///< Write block data
#define PICC_DECREMENT  0xC0  ///< Deduct from block value
#define PICC_INCREMENT  0xC1  ///< Add to block value
#define PICC_RESTORE    0xC2  ///< Restore data to the internal buffer
#define PICC_TRANSFER   0xB0  ///< Transfer internal buffer data to a block
#define PICC_HALT       0x50  ///< Halt communication with the card

//******************************************************************
//                        RC522 Register Definitions
//******************************************************************
// PAGE 0
#define RFU00           0x00  ///< Reserved
#define CommandReg      0x01  ///< Command register
#define ComIEnReg       0x02  ///< Interrupt enable register
#define DivlEnReg       0x03  ///< Interrupt request (IRQ) enable register
#define ComIrqReg       0x04  ///< Interrupt request (IRQ) status register
#define DivIrqReg       0x05  ///< Second IRQ status register
#define ErrorReg        0x06  ///< Error status register
#define Status1Reg      0x07  ///< Communication status register 1
#define Status2Reg      0x08  ///< Communication status register 2
#define FIFODataReg     0x09  ///< FIFO buffer input/output register
#define FIFOLevelReg    0x0A  ///< Indicates the number of bytes in FIFO
#define WaterLevelReg   0x0B  ///< FIFO water-level trigger setting
#define ControlReg      0x0C  ///< Control register for module settings
#define BitFramingReg   0x0D  ///< Bit framing for data transmission
#define CollReg         0x0E  ///< Collision detection register
#define RFU0F           0x0F  ///< Reserved

// PAGE 1
#define RFU10           0x10  ///< Reserved
#define ModeReg         0x11  ///< Mode configuration register
#define TxModeReg       0x12  ///< Transmit mode configuration
#define RxModeReg       0x13  ///< Receive mode configuration
#define TxControlReg    0x14  ///< Transmitter control register
#define TxASKReg        0x15  ///< Transmit ASK modulation settings
#define TxSelReg        0x16  ///< Transmitter input selection
#define RxSelReg        0x17  ///< Receiver input selection
#define RxThresholdReg  0x18  ///< Receiver threshold settings
#define DemodReg        0x19  ///< Demodulator settings
#define RFU1A           0x1A  ///< Reserved
#define RFU1B           0x1B  ///< Reserved
#define MifareReg       0x1C  ///< MIFARE-specific settings
#define RFU1D           0x1D  ///< Reserved
#define RFU1E           0x1E  ///< Reserved
#define SerialSpeedReg  0x1F  ///< UART baud rate configuration

// PAGE 2
#define RFU20           0x20  ///< Reserved
#define CRCResultRegM   0x21  ///< CRC result (MSB)
#define CRCResultRegL   0x22  ///< CRC result (LSB)
#define RFU23           0x23  ///< Reserved
#define ModWidthReg     0x24  ///< Modulation width setting
#define RFU25           0x25  ///< Reserved
#define RFCfgReg        0x26  ///< RF configuration
#define GsNReg          0x27  ///< Conductance settings
#define CWGsCfgReg      0x28  ///< CW conductance settings
#define ModGsCfgReg     0x29  ///< Modulation conductance settings
#define TModeReg        0x2A  ///< Timer mode settings
#define TPrescalerReg   0x2B  ///< Timer prescaler settings
#define TReloadRegH     0x2C  ///< Timer reload value (MSB)
#define TReloadRegL     0x2D  ///< Timer reload value (LSB)
#define TCounterValueRegH  0x2E  ///< Timer counter value (MSB)
#define TCounterValueRegL  0x2F  ///< Timer counter value (LSB)

// PAGE 3
#define RFU30           0x30  ///< Reserved
#define TestSel1Reg     0x31  ///< Test signal selection register 1
#define TestSel2Reg     0x32  ///< Test signal selection register 2
#define TestPinEnReg    0x33  ///< Enable test pin register
#define TestPinValueReg 0x34  ///< Test pin value register
#define TestBusReg      0x35  ///< Test bus settings
#define AutoTestReg     0x36  ///< Self-test settings
#define VersionReg      0x37  ///< Version register
#define AnalogTestReg   0x38  ///< Analog test settings
#define TestDAC1Reg     0x39  ///< Test DAC1 settings
#define TestDAC2Reg     0x3A  ///< Test DAC2 settings
#define TestADCReg      0x3B  ///< Test ADC settings
#define RFU3C           0x3C  ///< Reserved
#define RFU3D           0x3D  ///< Reserved
#define RFU3E           0x3E  ///< Reserved
#define RFU3F           0x3F  ///< Reserved

//******************************************************************
//                        Error Code Definitions
//******************************************************************
#define MI_OK                    0x00  ///< Operation successful
#define MI_ERR                   0xFE  ///< General error
#define MI_NOTAGERR              0xFF  ///< No card detected
#define MI_AUTHERR               0xFC  ///< Authentication error
#define MI_CRCERR                0xFE  ///< CRC error
#define MI_COLLERR               0xE8  ///< Collision error
#define MI_INTERFACEERR          0xE7  ///< Communication interface error
#define MI_ACCESSTIMEOUT         0xE5  ///< Card access timeout
#define MI_FIFOERR               0x94  ///< FIFO buffer overflow
#define MI_WRONG_PARAMETER_VALUE 0xC5  ///< Invalid parameter value
#define MI_UNKNOWN_COMMAND       0xE9  ///< Unknown command error

#define I2C_ADDR  0x28 ///< Default I2C address for the RFID module.

/**
 * @class DFRobot_FM17550
 * @brief Class for controlling the RC522 RFID card reader module.
 */
class DFRobot_FM17550 {
public:
  /**
   * @brief Constructor for the card reader.
   * @param pWire Pointer to the I2C interface (default: Wire).
   * @param addr I2C address of the card reader (default: 0x28).
   */
  DFRobot_FM17550(TwoWire *pWire = &Wire, uint8_t addr = I2C_ADDR);

  /**
   * @brief Initializes the card reader module.
   * @param rstPin Pin number for resetting the module (default: 0).
   * @return Returns 0 on success; otherwise, returns an error code.
   */
  char begin();

  /**
   * @brief Sets the authentication key for communication with the card.
   * @param key Pointer to an array containing the 6-byte key.
   */
  void setKey(uint8_t *key);
  
  /*!
   * @fn scan(String nfcuid)
   * @brief Scan to determine whether there is a NFC smart card/tag with the specified UID.
   * @param nfcuid UID of the NFC card.
   * @return Boolean type, the result of operation
   * @retval true Finds a card with a specific UID
   * @retval false The card with a specific UID was not found
   */
  bool scan(String nfcuid);

  /*!
   * @fn scan(void)
   * @brief Scan to determine whether there is a NFC smart card/tag.
   * @return Boolean type, the result of operation
   * @retval true means find out a MIFARE Classic card.
   * @retval false no card
   */
  bool  scan(void);

  /*!
   * @fn readUid
   * @brief Obtain the UID of the card .
   * @return UID of the card.
   */
  String  readUid();

   /*!
    * @fn readData(int block, uint8_t offset)
    * @brief Read a byte from a specified block of a MIFARE Classic NFC smart card/tag.
    * @param block The number of the block to read from.
    * @param offset The offset of the block.
    * @return A byte read from the card.
    */
  uint8_t readData(uint8_t block, uint8_t offset);

  /*!
   * @fn readData(uint8_t* buffer, uint8_t block)
   * @brief Read a block from a MIFARE Classic NFC smart card/tag (16 bytes each block).
   * @param buffer The buffer of the read data.
   * @param block The number of the block to read from.
   * @return Status code.
   * @retval 1 successfully read data
   * @retval -1 Failed to read data
   */
  String readData(uint8_t block);

  /*!
   * @fn writeData(int block, uint8_t num, uint8_t data)
   * @brief Write a byte to a MIFARE Classic NFC smart card/tag.
   * @param block The number of pages you want to writes the data.
   * @param num The offset of the data.
   * @param data The byte to be written.
   */
  char writeData(uint8_t block, uint8_t num, uint8_t data);

  /*!
   * @fn writeData(int block, uint8_t data[])
   * @brief Write a block to a MIFARE Classic NFC smart card/tag..
   * @param block The number of the block to write to.
   * @param data The buffer of the data to be written.
   */
  char  writeData(uint8_t block, uint8_t data[]);

private:
  /**
   * @brief Halts the currently connected RFID card.
   * @return Returns 0 on success; otherwise, returns an error code.
   */
  char pcdHalt(void);

  /**
   * @brief Activates the RFID module antenna.
   * @details This function enables the RF field required for card communication.
   */
  void pcdAntennaOn(void);
  
  /**
   * @brief Deactivates the RFID module antenna.
   * @details This function disables the RF field, reducing power consumption.
   */
  void pcdAntennaOff(void);
  
  /**
   * @brief Sends a request to detect a card.
   * @param req_code Type of request (e.g., PICC_REQIDL for idle cards).
   * @param pTagType Pointer to a buffer for storing the detected card type.
   * @return Returns 0 on success; otherwise, returns an error code.
   */
  char pcdRequest(unsigned char req_code, unsigned char *pTagType);
  
  /**
   * @brief Performs anti-collision detection for multiple cards in the field.
   * @param pSnr Pointer to a buffer for storing the detected card serial number.
   * @return Returns 0 on success; otherwise, returns an error code.
   */
  char pcdAnticoll(unsigned char *pSnr);
  
  /**
   * @brief Selects a card for communication.
   * @param pSnr Pointer to an array containing the card serial number.
   * @return Returns 0 on success; otherwise, returns an error code.
   */
  char pcdSelect(unsigned char *pSnr);
  
  /**
   * @brief Authenticates a specific card block with the given key.
   * @param auth_mode Authentication mode (e.g., PICC_AUTHENT1A for key A).
   * @param addr Address of the block to authenticate.
   * @param pKey Pointer to the 6-byte key array.
   * @param pSnr Pointer to the card's serial number array.
   * @return Returns 0 on success; otherwise, returns an error code.
   */
  char pcdAuthState(unsigned char auth_mode, unsigned char addr, unsigned char *pKey, unsigned char *pSnr);
  
  /**
   * @brief Reads data from a specific card block.
   * @param addr Address of the block to read.
   * @param pData Pointer to a buffer for storing the read data.
   * @return Returns 0 on success; otherwise, returns an error code.
   */
  char pcdRead(unsigned char addr, unsigned char *pData);
  
  /**
   * @brief Writes data to a specific card block.
   * @param addr Address of the block to write.
   * @param pData Pointer to an array containing the data to write.
   * @return Returns 0 on success; otherwise, returns an error code.
   */
  char pcdWrite(unsigned char addr, unsigned char *pData);
  
  /**
   * @brief Calculates the CRC checksum for the given data.
   * @param pIndata Pointer to the input data array.
   * @param len Length of the input data array.
   * @param pOutData Pointer to a buffer for storing the CRC checksum.
   */
  void calulateCRC(unsigned char *pIndata, unsigned char len, unsigned char *pOutData);
  
  /**
   * @brief Writes directly to a card reader register.
   * @param Address Register address.
   * @param value The value to write.
   */
  void writeRawRC(unsigned char Address, unsigned char value);
  
  /**
   * @brief Reads directly from a card reader register.
   * @param Address Register address.
   * @return The value read from the register.
   */
  unsigned char readRawRC(unsigned char Address);
  
  /**
   * @brief Sets specific bits in a card reader register.
   * @param reg Register address.
   * @param mask Bit mask for the bits to set.
   */
  void setBitMask(unsigned char reg, unsigned char mask);
  
  /**
   * @brief Clears specific bits in a card reader register.
   * @param reg Register address.
   * @param mask Bit mask for the bits to clear.
   */
  void clearBitMask(unsigned char reg, unsigned char mask);
  
  /**
   * @brief Enables the antenna test mode for debugging or development purposes.
   */
  void pcdAntennaTestOn(void);
  
  /**
   * @brief Puts the card reader into a low-power mode to conserve energy.
   */
  void softLowPower();
  
  /**
   * @brief Establishes a connection with a card.
   * @return True if a card is successfully connected, false otherwise.
   */
  bool connect();
  
  /**
   * @brief Sends a halt command to a connected card.
   * @return Returns 0 on success; otherwise, returns an error code.
   */
  char MIFHalt(void);                            
  
  /**
   * @brief Reads a block of data from a specified card memory block.
   * @param Block The memory block address on the card (0-63 for Mifare cards).
   * @param Buf Pointer to an array where the 16-byte block data will be stored.
   * @return Returns MI_OK (0x00) on success; otherwise, returns an error code.
   * @note Before calling this function, the block must be authenticated using the pcdAuthState function.
   *       The buffer provided by Buf must be at least 16 bytes to store the full block data.
   */
  char readBlock(unsigned char Block, unsigned char *Buf);
  
  /**
   * @brief Writes a block of data to a specified card memory block.
   * @param Block The memory block address on the card (0-63 for Mifare cards).
   * @param data Pointer to an array of 16 bytes containing the data to write.
   * @return Returns MI_OK (0x00) on success; otherwise, returns an error code.
   * @note Before calling this function, the block must be authenticated using the pcdAuthState function.
   */
  char writeBlock(unsigned char Block, uint8_t *data);
  
  /**
   * @brief Sends a command to the RC522 and handles the response.
   * @param Command The command to send to the RC522 (e.g., PCD_TRANSCEIVE, PCD_AUTHENT).
   * @param pInData Pointer to the input data array to send to the RC522.
   * @param InLenByte The length of the input data array in bytes.
   * @param pOutData Pointer to a buffer for storing the received data.
   * @param pOutLenBit Pointer to a variable to store the length of the received data in bits.
   * @return Returns MI_OK (0x00) on success; otherwise, returns an error code.
   * @details This function handles low-level communication with the RC522 module, 
   *          including transmitting data and receiving the response.
   */
  char pcdComMF522(unsigned char Command, unsigned char *pInData,
                   unsigned char InLenByte, unsigned char *pOutData,
                   unsigned int *pOutLenBit);
  
  /**
   * @brief Performs arithmetic operations (increment, decrement, restore) on a card memory block.
   * @param dd_mode The operation mode (e.g., PICC_INCREMENT, PICC_DECREMENT, PICC_RESTORE).
   * @param addr The memory block address to perform the operation on.
   * @param pValue Pointer to a 4-byte array containing the value to use for the operation.
   * @return Returns MI_OK (0x00) on success; otherwise, returns an error code.
   * @note The result of the operation is stored temporarily in the card's buffer
   *       and must be committed to a memory block using the PICC_TRANSFER command.
   */
  char pcdValue(unsigned char dd_mode, unsigned char addr, unsigned char *pValue);
  
  /**
   * @brief Copies the value from one memory block to another on the card.
   * @param sourceaddr The address of the source memory block.
   * @param goaladdr The address of the destination memory block.
   * @return Returns MI_OK (0x00) on success; otherwise, returns an error code.
   * @note This function transfers the value directly between blocks without involving the host.
   */
  char pcdBakValue(unsigned char sourceaddr, unsigned char goaladdr);

public:
  String cardType;

private:
	uint8_t _addr = 0x28;
	TwoWire *_pWire;
  char KK[8]; 
  unsigned char RF_Buffer[18]; 
  unsigned char UID[4];
  unsigned char Password_Buffer[6]= {0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF};
  unsigned char des_on; 
	};

#endif // DFRobot_FM17550_H