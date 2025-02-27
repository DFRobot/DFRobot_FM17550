# DFRobot_FM17550

* [中文版](./README_CN.md)

![正反面svg效果图](../resources/images/featured.png)

## Product Link（ https://www.dfrobot.com/ ）
    
## Table of Contents

* [Summary](#summary)
* [Installation](#installation)
* [Methods](#methods)
* [Compatibility](#compatibility)
* [History](#history)
* [Credits](#credits)

## Summary
This directory provides the methods for driving this module on the line board K10 and the line board M10. micropython is the driver for K10 and python is the driver for M10

## Methods
```Python
    #K10 MicoPython init method
    def __init__(self,scl=48,sda=47,bus=0)
        """!
        @brief Module I2C communication init
        @param i2c_addr - I2C communication address
        @param scl - I2C scl pin
        @param sda - I2C sda pin
        @param bus - I2C bus
        """
    #M10 Python init method
    def __init__(self,bus=4)
        """!
        @brief Module I2C communication init
        @param i2c_addr - I2C communication address
        @param bus - I2C bus
        """
    def begin(self)
        """!
        @brief Function begin.
        """
    def scan(uuid="")
        """!
        @brief scans to determine if there is an NFC smart card/label.
        @param uuid - When the uuid is set, this method is used to scan the card of the uuid. If the UUID is not set, it detects whether the nfc card exists
        @return Boolean type, operation result
        @retval True: Find the card
        @retval False: No card
        """
    def read_uid()
        """!
        @brief Obtain the UID of the card.
        @return UID of the card.
        """
    def write_data(self, block,data,index=0)
        """!
        @brief writes data to MIFARE classic NFC smart card/tag.
        @param block - The number of the block to be written
        @param data - Data to be written, when index is 0, indicates the size of a block to be written, when index is greater than 0, indicates a byte to be written
        @param index - Data offset (1-16)
        @return Boolean type, operation result
        """
    def read_data(self, block, index=None)
        """!
        @brief reads data from the specified block of the MIFARE classic NFC smart card/tag.
        @param block - The number of the block to read
        @param index - Data offset (1-16). When index is None, it indicates the size of a block to be read. When index is greater than 0, it indicates a byte to be read
        @return Indicates the read data
        """
```

## Compatibility

MCU                | Work Well    | Work Wrong   | Untested    | Remarks
------------------ | :----------: | :----------: | :---------: | -----
unihiker K10        |      √       |              |             | 
unihiker M10        |      √       |              |             | 


## History

- Date 2025-02-08
- Version V0.1
## Credits
Written by fary(feng.yang@dfrobot.com), 2025.02.08 (Welcome to our [website](https://www.dfrobot.com/))