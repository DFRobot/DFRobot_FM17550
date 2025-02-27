/*!
 *@file scan.ino
 *@brief nfc scan
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
void setup(){
  Serial.begin(115200);
  card.begin();
}
void loop(){
  if(card.scan()){
    Serial.print("Card Type is: ");
    Serial.println(card.cardType);
    Serial.print("Card ID is: ");
    Serial.println(card.readUid());
  }
  delay(1000);
}