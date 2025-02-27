# DFRobot_FM17550

* [English Version](./README.md)



![正反面svg效果图](../resources/images/featured.png)

## 产品链接（ https://www.dfrobot.com.cn/ ）



## 目录

* [简介](#简介)
* [方法](#方法)
* [兼容性](#兼容性)
* [历史](#历史)
* [贡献者](#贡献者)

## 简介
本目录提供了行空板K10已经行空板M10驱动本模块的办法，micropython为K10的驱动方法，python为M10的驱动方法
## 方法
```Python
    #行空板K10 MicoPython的init办法
    def __init__(self,scl=48,sda=47,bus=0)
      """!
      @brief 模块I2C通信初始化
      @param scl - I2C scl 引脚
      @param sda - I2C sda 引脚
      @param bus - I2C 总线号
      """
    #行空板M10 Python的init办法
    def __init__(self,bus=4)
        """!
        @brief 模块I2C通信初始化
        @param bus - I2C 总线号
        """
    def begin(self)
        """!
        @brief 模块初始化.
        """
    def scan(uuid="")
        """!
        @brief 扫描确定是否有NFC智能卡/标签。
        @param uuid - 当uuid被设置值时，该方法用来扫描该uuid的卡，如没有设置则检测是否存在nfc 卡
        @return Boolean 类型, 操作结果
        @retval True：找到卡
        @retval False：无卡
        """
    def read_uid()
        """!
        @brief 获取卡的UID。
        @return 卡的UID。
        """
    def write_data(self, block,data,index=0)
        """!
        @brief 写数据到MIFARE经典NFC智能卡/标签。
        @param block - 要写入的块的编号
        @param data - 要写入的数据，当index为0时，表示写一个块的大小，当index大于0时，表示写入一个字节
        @param index - 数据偏移量（1-16）
        @return Boolean 类型, 操作结果
        """
    def read_data(self, block, index=None)
        """!
        @brief 从MIFARE经典NFC智能卡/标签的指定块中读取数据。
        @param block - 要读取的块的编号
        @param index - 数据偏移量（1-16）。当index为None时，表示读一个块的大小，当index大于0时，表示读一个字节
        @return 读取到的数据
        """
```


## 兼容性

开发板                | 表现良好	|表现异常	|未测试	|备注 |
------------------ | :----------: | :----------: | :---------: | -----
行空板K10        |      √       |              |             | 
行空板M10        |      √       |              |             | 

## 历史

- Date 2025-02-08
- Version V0.1


## 贡献者

Written by fary(feng.yang@dfrobot.com), 2025.02.08 (Welcome to our [website](https://www.dfrobot.com/))
