/*!
 *@file Read-writeTest.ino
 *@brief nfc read and write test
 *@details  
 *@copyright   Copyright (c) 2024 DFRobot Co.Ltd (http://www.dfrobot.com)
 *@license     The MIT license (MIT)
 *@author [fary](feng.yang@dfrobot.com)
 *@version  V1.0
 *@date  2025-02-07
 *@https://github.com/DFRobot/DFRobot_FM17550
*/


#include <DFRobot_FM17550.h>
DFRobot_FM17550 card;
uint8_t RevBuffer[16]={1,2,3,4,7};
void setup(){
  Serial.begin(115200);
  card.begin();
}
void loop(){
  if(card.scan("fc06ac91")){
    Serial.print("Card Type is: ");
    Serial.println(card.cardType);
    Serial.print("Card ID is: ");
    Serial.println(card.readUid());
    if(card.writeData(2,RevBuffer)==MI_OK){
      Serial.print("Successful data write!: ");
      Serial.println(card.readData(2));
    }
    if(card.writeData(2,1,0xDF)==MI_OK){
      Serial.print("Change the first byte of data block 1 to:");
      Serial.println(card.readData(2,1));
    }
  }
  delay(1000);
}
