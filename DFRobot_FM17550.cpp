/*!
 * @file DFRobot_FM17550.cpp
 * @brief Define the basic structure of class DFRobot_FM17550 
 * @copyright   Copyright (c) 2024 DFRobot Co.Ltd (http://www.dfrobot.com)
 * @licence     The MIT License (MIT)
 * @author [fary](feng.yang@dfrobot.com)
 * @version  V1.0
 * @date  2025-02-08
 * @get from https://www.dfrobot.com
 * @https://github.com/cdjq/DFRobot_FM17550
 */
#include <DFRobot_FM17550.h>

String convertToHexString(const unsigned char *uid, int length) {
  String hexString = "";
  for (int i = 0; i < length; i++) {
      // Converts each byte to two bits of hexadecimal and appends to the string
      if (uid[i] < 0x10) { // If it is less than 0x10, fill in the zero
          hexString += "0";
      }
      hexString += String(uid[i], HEX);

      if ((i < length - 1) &&(length > 4)) { // Add a space after each byte except for the last byte
        hexString += " ";
      }
  }

  hexString.toUpperCase(); // Convert letters to uppercase (optional)
  return hexString;
}

bool DFRobot_FM17550::scan(void)
{
  pcdHalt();
  uint8_t Temp[4];
  bool ret=true;

  memset(UID, 0, sizeof(UID));

  if(pcdRequest(0x52,Temp)==0){
    if(Temp[0]==0x04&&Temp[1]==0x00){ 
      cardType = "MFOne-S50";
    }else if(Temp[0]==0x02&&Temp[1]==0x00){
      cardType = "MFOne -S70";
    }else if(Temp[0]==0x44&&Temp[1]==0x00){
      cardType = "MF-UltraLight";
    }else if(Temp[0]==0x08&&Temp[1]==0x00){
      cardType = "MF-Pro";
    }else if(Temp[0]==0x44&&Temp[1]==0x03){
      cardType = "MF Desire";
    }else{
      cardType = "Unknown";
      ret =false;
    }
  }else{
    cardType = "No Card";
    ret =false;
    UID[0] = 0xFF;
  }
  if(ret){
    if( (pcdAnticoll(UID)!=0)){
      ret = false;
    }else{
      if((pcdSelect(UID)!=0) ){
        ret = false; 
      }
    }
  }
  return ret;
}

bool DFRobot_FM17550::scan(String nfcUid)
{
  bool ret=false;
  if(scan()){
    String UIDStr=convertToHexString(UID,4);
    UIDStr.toLowerCase() ;
    String nfcUid_ = nfcUid;
    nfcUid_.toLowerCase() ;
    if(UIDStr == nfcUid_){
      ret = true;
    }
  }
  return ret;
}

String DFRobot_FM17550::readUid()
{	
  scan();
  if (UID[0] == 0xFF) {
    return "No Card";
  }
  return convertToHexString(UID,4);   
}

uint8_t DFRobot_FM17550::readData(uint8_t block, uint8_t offset)
{
  scan();
  if (UID[0] == 0xFF) {
    return 0;
  }
  uint8_t Buf[16];
  if(offset>0){
    readBlock(block,Buf);
  }
  return Buf[offset-1];
}

String DFRobot_FM17550::readData(uint8_t block)
{
  scan();
  if (UID[0] == 0xFF) {
    return "No Card";
  }
  uint8_t Buf[16]={0};
  readBlock(block,Buf);

  return convertToHexString(Buf,16);
}

char DFRobot_FM17550::writeData(uint8_t block, uint8_t num, uint8_t data)
{
  char ret = MI_ERR;
  if( (num>0) && (block>0) ){
    uint8_t Buf[16];
    readBlock(block,Buf);
    Buf[num-1]=data;
    ret = writeBlock(block,Buf);
  }
  return ret;
}
char DFRobot_FM17550::writeData(uint8_t block, uint8_t data[])
{
  char ret = MI_ERR;
  if( block>0 ){
    ret = writeBlock(block,data);
  }
  return ret;
}
DFRobot_FM17550::DFRobot_FM17550(TwoWire * pWire, uint8_t addr)
{
	_pWire = pWire;
	_addr = addr;	
}

unsigned char DFRobot_FM17550::readRawRC(unsigned char Address)
{
  uint8_t value;
  _pWire->beginTransmission(_addr);
  _pWire->write(Address);
  _pWire->endTransmission();
  
  _pWire->requestFrom(_addr,(uint8_t)1);
  value = _pWire->read();
  
  return value;
}

void DFRobot_FM17550::writeRawRC(unsigned char Address, unsigned char value)
{  
  _pWire->beginTransmission(_addr);
  _pWire->write(Address);
  _pWire->write(value);
  _pWire->endTransmission();
}


void DFRobot_FM17550::setBitMask(unsigned char reg,unsigned char mask)  
{
  char tmp = 0x00;
  tmp = readRawRC(reg)| mask;
  writeRawRC(reg,tmp | mask);  
}


void DFRobot_FM17550::clearBitMask(unsigned char reg,unsigned char mask)  
{
  char tmp = 0x00;
  tmp = readRawRC(reg)&(~mask);
  writeRawRC(reg, tmp);  
} 

char DFRobot_FM17550::begin()
{
  _pWire->begin();
  writeRawRC(CommandReg,PCD_RESETPHASE);
  delay(1);
  writeRawRC(ModeReg,0x3D);
  writeRawRC(TReloadRegL,30);
  writeRawRC(TReloadRegH,0);
  writeRawRC(TModeReg,0x8D);
  writeRawRC(TPrescalerReg,0x3E);
  pcdAntennaOn();   
  return MI_OK; 
}

void DFRobot_FM17550::pcdAntennaOn()
{
  unsigned char i;
  writeRawRC(TxASKReg,0x40);
  delay(10);
  i = readRawRC(TxControlReg);
  if(!(i&0x03)){
    setBitMask(TxControlReg, 0x03);
  }
  i=readRawRC(TxASKReg);
}

void DFRobot_FM17550::pcdAntennaTestOn()
{
  writeRawRC(TxControlReg,0x02);
}


void DFRobot_FM17550::pcdAntennaOff()
{
  clearBitMask(TxControlReg, 0x03);
}

char DFRobot_FM17550::pcdComMF522(unsigned char Command  ,unsigned char *pInData ,unsigned char InLenByte, 
                                     unsigned char *pOutData,unsigned int  *pOutLenBit)
{
  char status = MI_ERR;
  unsigned char irqEn = 0x00;
  unsigned char waitFor = 0x00;
  unsigned char lastBits;
  unsigned char n;
  unsigned int i;
  switch (Command){
    case PCD_AUTHENT:
      irqEn   = 0x12;
      waitFor = 0x10;
      break;
    case PCD_TRANSCEIVE:
      irqEn   = 0x77;
      waitFor = 0x30;
      break;
    default:
      break;
  }
  writeRawRC(ComIEnReg,irqEn|0x80); 
  clearBitMask(ComIrqReg,0x80);
  writeRawRC(CommandReg,PCD_IDLE);
  setBitMask(FIFOLevelReg,0x80); 
  for(i=0; i<InLenByte; i++){
    writeRawRC(FIFODataReg,pInData[i]);
  }
  writeRawRC(CommandReg, Command); 
  if(Command == PCD_TRANSCEIVE){
    setBitMask(BitFramingReg,0x80);
  }
  i = 6000; 
  do 
  {
    n = readRawRC(ComIrqReg);
    i--;
  }
  while((i!=0)&&!(n&0x01)&&!(n&waitFor));
  clearBitMask(BitFramingReg,0x80);
  if(i!=0){
    if(!(readRawRC(ErrorReg)&0x1B)){
      status = MI_OK;
      if (n&irqEn&0x01)
        status = MI_NOTAGERR;
      if(Command==PCD_TRANSCEIVE){
        n = readRawRC(FIFOLevelReg);
        lastBits = readRawRC(ControlReg)&0x07;
        if(lastBits){
          *pOutLenBit = (n-1)*8 + lastBits;
        }else{
          *pOutLenBit = n*8;
        }
        if(n==0){
          n = 1;
        }
        if(n>MAXRLEN){
          n = MAXRLEN;
        }
        for (i=0; i<n; i++){
          pOutData[i] = readRawRC(FIFODataReg); 
        }
      }
    }else{
      status = MI_ERR;  
    }      
  }
  setBitMask(ControlReg,0x80);
  writeRawRC(CommandReg,PCD_IDLE); 
  return status;
}

char DFRobot_FM17550::pcdRequest(unsigned char req_code,unsigned char *pTagType)
{
  char status;  
  unsigned int  unLen;
  unsigned char ucComMF522Buf[MAXRLEN]; 

  clearBitMask(Status2Reg,0x08);
  writeRawRC(BitFramingReg,0x07);
  setBitMask(TxControlReg,0x03);

  ucComMF522Buf[0] = req_code;

  status = pcdComMF522(PCD_TRANSCEIVE,ucComMF522Buf,1,ucComMF522Buf,&unLen);
  if((status == MI_OK) && (unLen == 0x10)){    
    *pTagType = ucComMF522Buf[0];
    *(pTagType+1) = ucComMF522Buf[1];
  }else{
    status = MI_ERR;
  }
  return status;
}

char DFRobot_FM17550::pcdAnticoll(unsigned char *pSnr)
{
  char status;
  unsigned char i,snr_check=0;
  unsigned int  unLen;
  unsigned char ucComMF522Buf[MAXRLEN]; 
  
  clearBitMask(Status2Reg,0x08);
  writeRawRC(BitFramingReg,0x00);
  clearBitMask(CollReg,0x80);
  
  ucComMF522Buf[0] = PICC_ANTICOLL1;
  ucComMF522Buf[1] = 0x20;
  
  status = pcdComMF522(PCD_TRANSCEIVE,ucComMF522Buf,2,ucComMF522Buf,&unLen);
  
  if (status == MI_OK){
  	for (i=0; i<4; i++){   
      *(pSnr+i) = ucComMF522Buf[i];
      snr_check ^= ucComMF522Buf[i];
    }
    if (snr_check != ucComMF522Buf[i]){
      status = MI_ERR;    
    }
  }
  setBitMask(CollReg,0x80);
  return status;
}

char DFRobot_FM17550::pcdSelect(unsigned char *pSnr)
{
  char status;
  unsigned char i;
  unsigned int unLen;
  unsigned char ucComMF522Buf[MAXRLEN]; 
  
  ucComMF522Buf[0] = PICC_ANTICOLL1;
  ucComMF522Buf[1] = 0x70;
  ucComMF522Buf[6] = 0;
  for (i=0; i<4; i++){
  	ucComMF522Buf[i+2] = *(pSnr+i);
  	ucComMF522Buf[6]  ^= *(pSnr+i);
  }
  calulateCRC(ucComMF522Buf,7,&ucComMF522Buf[7]);
  
  clearBitMask(Status2Reg,0x08);
  
  status = pcdComMF522(PCD_TRANSCEIVE,ucComMF522Buf,9,ucComMF522Buf,&unLen);
  
  if((status == MI_OK) && (unLen == 0x18)){
    status = MI_OK;
  }else{
    status = MI_ERR;
  }
  return status;
}

char DFRobot_FM17550::pcdAuthState(unsigned char auth_mode,unsigned char addr,unsigned char *pKey,unsigned char *pSnr)
{
    char status;
    unsigned int unLen;
    unsigned char i,ucComMF522Buf[MAXRLEN]; 

    ucComMF522Buf[0] = auth_mode;
    ucComMF522Buf[1] = addr;
    for (i=0; i<6; i++){
      ucComMF522Buf[i+2] = *(pKey+i);
    }
    for (i=0; i<4; i++){
      ucComMF522Buf[i+8] = *(pSnr+i);
    }
    
    status = pcdComMF522(PCD_AUTHENT,ucComMF522Buf,12,ucComMF522Buf,&unLen);
    if ((status != MI_OK) || (!(readRawRC(Status2Reg) & 0x08))){
      status = MI_ERR; 
    }
    
    return status;
}

char DFRobot_FM17550::pcdRead(unsigned char addr,unsigned char *pData)
{
  char status;
  unsigned int  unLen;
  unsigned char i,ucComMF522Buf[MAXRLEN]; 
  
  ucComMF522Buf[0] = PICC_READ;
  ucComMF522Buf[1] = addr;
  calulateCRC(ucComMF522Buf,2,&ucComMF522Buf[2]);   
  status = pcdComMF522(PCD_TRANSCEIVE,ucComMF522Buf,4,ucComMF522Buf,&unLen);
  if ((status == MI_OK) && (unLen == 0x90)){
    for (i=0; i<16; i++){
      *(pData+i) = ucComMF522Buf[i];   
    }
  }else{
    status = MI_ERR;  
  }
  return status;
}

void DFRobot_FM17550::setKey(uint8_t *key)
{
  //Password_Buffer[6]  
  for(uint8_t i = 0;i<6 ;i++){
    Password_Buffer[i] = key[i];
	}
}

bool DFRobot_FM17550::connect()
{
  uint8_t buffer[6];
  if(pcdAnticoll(buffer) == 0){
    memcpy(UID,buffer,5);
    if(pcdSelect(buffer) == 0){
      return true;
    }
    return false;
  }
}

char DFRobot_FM17550::readBlock(unsigned char Block,unsigned char *Buf)
{
  char result;
  //pcdAuthState
  result = pcdAuthState(0x60,Block,Password_Buffer,UID);
  if(result!=MI_OK){
    return result;
  }
  result = pcdRead(Block,Buf);
  if(result!=MI_OK){
    return result;
  }    
  return 0; 
}

char DFRobot_FM17550::pcdWrite(unsigned char addr,unsigned char *pData)
{
  char status;
  unsigned int unLen;
  unsigned char i,ucComMF522Buf[MAXRLEN]; 
    
  ucComMF522Buf[0] = PICC_WRITE;
  ucComMF522Buf[1] = addr;
  calulateCRC(ucComMF522Buf,2,&ucComMF522Buf[2]);
  status = pcdComMF522(PCD_TRANSCEIVE,ucComMF522Buf,4,ucComMF522Buf,&unLen);
  if(( status != MI_OK)||(unLen != 4)||((ucComMF522Buf[0]&0x0F)!= 0x0A)){
    status = MI_ERR;   
  }        
  if(status == MI_OK){
    for (i=0; i<16; i++){
      ucComMF522Buf[i] = *(pData+i);
    }
    calulateCRC(ucComMF522Buf,16,&ucComMF522Buf[16]);
    status = pcdComMF522(PCD_TRANSCEIVE,ucComMF522Buf,18,ucComMF522Buf,&unLen);
    if((status != MI_OK)||(unLen != 4 )||((ucComMF522Buf[0]&0x0F)!= 0x0A)){
      status = MI_ERR; 
    }
  }    
  return status;
}

char DFRobot_FM17550::writeBlock(unsigned char Block,uint8_t *data )
{
  char result;
  result = pcdAuthState(0x60,Block,Password_Buffer,UID);
  if(result!=MI_OK){
    return result;  
  }
  result = pcdWrite(Block,data);
  return result;
}

               
char DFRobot_FM17550::pcdValue(unsigned char dd_mode,unsigned char addr,unsigned char *pValue)
{
  char status;
  unsigned int unLen;
  unsigned char i,ucComMF522Buf[MAXRLEN]; 
  
  ucComMF522Buf[0] = dd_mode;
  ucComMF522Buf[1] = addr;
  calulateCRC(ucComMF522Buf,2,&ucComMF522Buf[2]);
  
  status = pcdComMF522(PCD_TRANSCEIVE,ucComMF522Buf,4,ucComMF522Buf,&unLen);  
  if((status != MI_OK) || (unLen != 4) || ((ucComMF522Buf[0] & 0x0F) != 0x0A)){
    status = MI_ERR;
  }
  
  if (status == MI_OK){
    for (i=0; i<16; i++){
      ucComMF522Buf[i] = *(pValue+i);
    }
    calulateCRC(ucComMF522Buf,4,&ucComMF522Buf[4]);
    unLen = 0;
    status = pcdComMF522(PCD_TRANSCEIVE,ucComMF522Buf,6,ucComMF522Buf,&unLen);
    if (status != MI_ERR){
      status = MI_OK;
    }
  }
  
  if (status == MI_OK){
    ucComMF522Buf[0] = PICC_TRANSFER;
    ucComMF522Buf[1] = addr;
    calulateCRC(ucComMF522Buf,2,&ucComMF522Buf[2]); 
  
    status = pcdComMF522(PCD_TRANSCEIVE,ucComMF522Buf,4,ucComMF522Buf,&unLen);  
    if ((status != MI_OK) || (unLen != 4) || ((ucComMF522Buf[0] & 0x0F) != 0x0A)){
      status = MI_ERR;
    }
  }
  return status;
}

char DFRobot_FM17550::pcdBakValue(unsigned char sourceaddr, unsigned char goaladdr)
{
  char status;
  unsigned int  unLen;
  unsigned char ucComMF522Buf[MAXRLEN]; 
  
  ucComMF522Buf[0] = PICC_RESTORE;
  ucComMF522Buf[1] = sourceaddr;
  calulateCRC(ucComMF522Buf,2,&ucComMF522Buf[2]);
  
  status = pcdComMF522(PCD_TRANSCEIVE,ucComMF522Buf,4,ucComMF522Buf,&unLen);
  
  if((status != MI_OK) || (unLen != 4) || ((ucComMF522Buf[0] & 0x0F) != 0x0A)){
    status = MI_ERR;  
  }
  
  if(status == MI_OK){
    ucComMF522Buf[0] = 0;
    ucComMF522Buf[1] = 0;
    ucComMF522Buf[2] = 0;
    ucComMF522Buf[3] = 0;
    calulateCRC(ucComMF522Buf,4,&ucComMF522Buf[4]);
  
    status = pcdComMF522(PCD_TRANSCEIVE,ucComMF522Buf,6,ucComMF522Buf,&unLen);
    if(status != MI_ERR){
      status = MI_OK;
    }
  }
  
  if(status != MI_OK){ 
    return MI_ERR;  
  }
  
  ucComMF522Buf[0] = PICC_TRANSFER;
  ucComMF522Buf[1] = goaladdr;
  
  calulateCRC(ucComMF522Buf,2,&ucComMF522Buf[2]);
  
  status = pcdComMF522(PCD_TRANSCEIVE,ucComMF522Buf,4,ucComMF522Buf,&unLen);
  
  if((status != MI_OK) || (unLen != 4) || ((ucComMF522Buf[0] & 0x0F) != 0x0A)){
    status = MI_ERR;
  }
  return status;
}

char DFRobot_FM17550::pcdHalt(void)
{
  char status;
  unsigned int  unLen;
  unsigned char ucComMF522Buf[MAXRLEN]; 
  
  ucComMF522Buf[0] = PICC_HALT;
  ucComMF522Buf[1] = 0;
  calulateCRC(ucComMF522Buf,2,&ucComMF522Buf[2]);
  
  status = pcdComMF522(PCD_TRANSCEIVE,ucComMF522Buf,4,ucComMF522Buf,&unLen);
  return status;
}


char DFRobot_FM17550::MIFHalt(void)
{
  char status;
  unsigned int  unLen;
  unsigned char ucComMF522Buf[MAXRLEN]; 
  
  ucComMF522Buf[0] = PICC_HALT;
  ucComMF522Buf[1] = 0;
  calulateCRC(ucComMF522Buf,2,&ucComMF522Buf[2]);
  
  status = pcdComMF522(PCD_TRANSCEIVE,ucComMF522Buf,4,ucComMF522Buf,&unLen);
  return status;  
}


void DFRobot_FM17550::softLowPower()
{
  writeRawRC(CommandReg,1<<4);
}


void DFRobot_FM17550::calulateCRC(unsigned char *pIndata,unsigned char len,unsigned char *pOutData)
{
  unsigned char i,n;
  clearBitMask(DivIrqReg,0x04);
  writeRawRC(CommandReg,PCD_IDLE);
  setBitMask(FIFOLevelReg,0x80);
  for (i=0; i<len; i++){
    writeRawRC(FIFODataReg, *(pIndata+i));
  }
  writeRawRC(CommandReg, PCD_CALCCRC);
  i = 0xFF;
  do {
    n = readRawRC(DivIrqReg);
    i--;
  }
  while ((i!=0) && !(n&0x04));
  pOutData[0] = readRawRC(CRCResultRegL);
  pOutData[1] = readRawRC(CRCResultRegM);
}