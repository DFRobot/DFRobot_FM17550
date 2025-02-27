# DFRobot_FM17550

* [English Version](./README.md)



![正反面svg效果图](https://github.com/cdjq/DFRobot_FM17550/raw/master/resources/images/featured.png)

## 产品链接（ https://www.dfrobot.com.cn/ ）



## 目录

* [简介](#简介)
* [安装](#安装)
* [方法](#方法)
* [兼容性](#兼容性)
* [历史](#历史)
* [贡献者](#贡献者)

## 简介



## 安装

1.要使用此库，首先下载库文件，将其粘贴到 `\Arduino\libraries` 目录中，然后打开示例文件夹并运行其中的示例。

## 方法
```c++
  /**
   * @brief 卡片阅读器的构造函数。
   * @param pWire I2C 接口的指针（默认：Wire）。
   * @param addr 卡片阅读器的 I2C 地址（默认：0x28）。
   */
  DFRobot_FM17550(TwoWire *pWire, uint8_t addr);
    
  /**
   * @brief 初始化读卡器模块。
   * @return 成功时返回0；否则，返回错误代码。
   */
  char begin();
  
  /*!
   * @fn scan(String nfcuid)
   * @brief 扫描确认是否有与指定UID匹配的NFC智能卡/标签。
   * @param nfcuid NFC卡的UID。
   * @return 操作的结果
   * @retval true：查找具有特定UID的卡
   * @retval false ：无卡
   */
  bool scan(String nfcuid);

  /*!
   * @fn scan(void)
   * @brief 扫描确定是否有NFC智能卡/标签。
   * @return 操作的结果
   * @retval true：找到一张MIFARE经典卡。
   * @retval false ：无卡
   */
  bool  scan(void);

  /*!
   * @fn readUid
   * @brief 获取卡的UID。
   * @return 卡的UID。
   */
  String  readUid();

   /*!
    * @fn readData(int block, uint8_t offset)
    * @brief 从MIFARE经典NFC智能卡/标签的指定块中读取一个字节。
    * @param block 要读取的块的编号。
    * @param offset 块的偏移量。
    * @return 从卡上读出的一个字节。
    */
  uint8_t readData(uint8_t block, uint8_t offset);

  /*!
   * @fn readData(uint8_t* buffer, uint8_t block)
   * @brief 从MIFARE经典NFC智能卡/标签读取一个块（每个块16字节）。
   * @param buffer 读取数据的缓冲区。
   * @param block 要读取的块的编号。
   * @return 读取到的数据，字符串类型
   */
  String readData(uint8_t block);

  /*!
   * @fn writeData(int block, uint8_t num, uint8_t data)
   * @brief 写一个字节到MIFARE经典NFC智能卡/标签。
   * @param block 要写入的块的编号。
   * @param num 数据偏移量。
   * @param data 要写入的字节。
   */
  char writeData(uint8_t block, uint8_t num, uint8_t data);

  /*!
   * @fn writeData(uint8_t block, uint8_t data[])
   * @brief  写一个块到MIFARE经典NFC智能卡/标签。
   * @param block 要写入的块的编号。
   * @param data 要写入数据的缓冲区。
   */
  char  writeData(uint8_t block, uint8_t data[]);

```


## 兼容性

MCU                | 表现良好	|表现异常	|未测试	|备注 |
------------------ | :----------: | :----------: | :---------: | -----
Arduino UNO        |      √       |              |             | 



## 历史

- Date 2025-02-08
- Version V0.1


## 贡献者

Written by fary(feng.yang@dfrobot.com), 2025.02.08 (Welcome to our [website](https://www.dfrobot.com/))
