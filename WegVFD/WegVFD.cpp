/*
Title
Description

Ricardo Mena C
ricardo@crcibernetica.com
http://crcibernetica.com

 License
 **********************************************************************************
 This program is free software; you can redistribute it 
 and/or modify it under the terms of the GNU General    
 Public License as published by the Free Software       
 Foundation; either version 3 of the License, or        
 (at your option) any later version.                    
                                                        
 This program is distributed in the hope that it will   
 be useful, but WITHOUT ANY WARRANTY; without even the  
 implied warranty of MERCHANTABILITY or FITNESS FOR A   
 PARTICULAR PURPOSE. See the GNU General Public        
 License for more details.                              
                                                        
 You should have received a copy of the GNU General    
 Public License along with this program.
 If not, see <http://www.gnu.org/licenses/>.
                                                        
 Licence can be viewed at                               
 http://www.gnu.org/licenses/gpl-3.0.txt

 Please maintain this license information along with authorship
 and copyright notices in any redistribution of this code
 **********************************************************************************
*/

#include "WegVFD.h"

WegVFD::WegVFD(){}//Constructor

WegVFD::WegVFD(HardwareSerial &rsSerial, uint32_t baud, uint8_t SSerialTxControl):rs485(&rsSerial){
  rs485->begin(baud);
  rs485->flush();
  this->SSerialTxControl = SSerialTxControl;
  digitalWrite(this->SSerialTxControl, RS485Receive);//Init 
  pinMode(this->SSerialTxControl, OUTPUT);   
  this->uchCRCHi = 0xFF; /* high byte of CRC initialized */
  this->uchCRCLo = 0xFF; /* low byte of CRC initialized */
}//end Constructor

uint8_t WegVFD::setFrecuency(uint8_t hertz){
  this->hertz = hertz;
  byte speed[8] = {0x01, 0x06, 0x02, 0xAB, 0x00, 0x00, 0xF9, 0x92};//Default 0Hz
  this->speedtwoHex(this->hertz, spdhex1, spdhex2);
  speed[4] = this->spdhex1;
  speed[5] = this->spdhex2;
  this->CRC16(speed, 6, uchCRCLo, uchCRCHi);//Find CRC+ and CRC-
  speed[6] = uchCRCLo;
  speed[7] = uchCRCHi;

  enableTransmit();
  delay(5);
  rs485->write(speed, 8);
  disableTransmit();
  delay(5);
  this->spdhex1 = this->spdhex2 = 0;
  return 1;
}//end frecuency

uint8_t WegVFD::getFrecuency(void){
  //Ask to cfw500?
  return hertz;
}


uint8_t WegVFD::enableTransmit(){
  digitalWrite(SSerialTxControl, RS485Transmit);  // Enable RS485 Transmit
  return digitalRead(SSerialTxControl);
}
uint8_t WegVFD::disableTransmit(){
  digitalWrite(SSerialTxControl, RS485Receive);  // Disable RS485 Transmit
  return digitalRead(SSerialTxControl);
}

/* The function returns the CRC as a unsigned short type */
//  unsigned char *puchMsg; /* message to calculate CRC upon */
//unsigned short usDataLen; /* quantity of bytes in message */
unsigned short WegVFD::CRC16(unsigned char *puchMsg, unsigned short usDataLen, unsigned char &uchCRCLo, unsigned char &uchCRCHi){
  //unsigned char uchCRCHi = 0xFF; /* high byte of CRC initialized */
  //unsigned char uchCRCLo = 0xFF; /* low byte of CRC initialized */  
  this->uchCRCHi = uchCRCHi= 0xFF;
  this->uchCRCLo = uchCRCLo = 0xFF;
 
  unsigned uIndex; /* will index into CRC lookup table */
  while (usDataLen--){ /* pass through message buffer */
    uIndex = uchCRCLo ^ *puchMsg++; /* calculate the CRC */
    this->uchCRCLo = uchCRCLo= uchCRCHi ^ auchCRCHi[uIndex];
    this->uchCRCHi = uchCRCHi = auchCRCLo[uIndex];
  }//end while
  return (uchCRCHi << 8 | uchCRCLo);
}//end CRC16

uint8_t WegVFD::twoHex(float variable, int &hex1, int &hex2){
  char temp[4] = {'0','0','0','0'};
  String temp2;
  String final[2] = {"", ""};

  int velocidad = round(variable);
  temp2 = String(velocidad, HEX);
  
  for(uint8_t i = 0; i<4; i++){
    temp[3-i] = temp2[(temp2.length()-1)-i];
  }//end for*/
  
  final[0] += String(temp[0])+String(temp[1]);
  final[1] += String(temp[2])+String(temp[3]);

  char const hex_chars[16] = { '0', '1', '2', '3', '4', '5', '6', '7', '8', '9', 'a', 'b', 'c', 'd', 'e', 'f' };

  int hex_1 = 0;
  int hex_2 = 0;
  for(uint8_t k = 0; k < 2; k++){
    for(uint8_t i = 0; i < final[k].length(); i++){
      for(uint8_t j = 0; j < 16; j++){
        if(final[k][i] == hex_chars[j]){
          if(k == 0){
            hex1 += round(j*pow(16, (final[k].length()-1)-i));
          }else{
            hex2 += round(j*pow(16, (final[k].length()-1)-i));
          }//end if
          break; 
        }//end if
      }//end for
    }//end for
  }//end for
  return 1;  
}//end speedTwoHex

uint8_t WegVFD::speedtwoHex(int mSpeed, int &hex1, int &hex2){
  char temp[4] = {'0','0','0','0'};
  String temp2;
  String final[2] = {"", ""};

  int velocidad = round(mSpeed*136.533);
  temp2 = String(velocidad, HEX);
  
  for(uint8_t i = 0; i<4; i++){
    temp[3-i] = temp2[(temp2.length()-1)-i];
  }//end for*/
  
  final[0] += String(temp[0])+String(temp[1]);
  final[1] += String(temp[2])+String(temp[3]);

  char const hex_chars[16] = { '0', '1', '2', '3', '4', '5', '6', '7', '8', '9', 'a', 'b', 'c', 'd', 'e', 'f' };

  int hex_1 = 0;
  int hex_2 = 0;
  for(uint8_t k = 0; k < 2; k++){
    for(uint8_t i = 0; i < final[k].length(); i++){
      for(uint8_t j = 0; j < 16; j++){
        if(final[k][i] == hex_chars[j]){
          if(k == 0){
            hex1 += round(j*pow(16, (final[k].length()-1)-i));
          }else{
            hex2 += round(j*pow(16, (final[k].length()-1)-i));
          }//end if
          break; 
        }//end if
      }//end for
    }//end for
  }//end for
  return 1;  
}//end twoHex


/*uint8_t WegVFD::speedControPHighTemp(float messuredValue, float minTemp, float setPoint, int minSpeed, int maxSpeed){
  int motorSpeed;

  
  if((messuredValue >= minTemp)&&(messuredValue <= setPoint)){
    motorSpeed = map(messuredValue, minTemp, setPoint, maxSpeed, minSpeed);
  }else if(messuredValue <= setPoint){
    motorSpeed = 45;
  }else if(messuredValue >= minTemp){
    motorSpeed = 0;  
  }//end if
  
  return motorSpeed;
}//end speedControPHighTemp

uint8_t WegVFD::speedControPLowTemp(float messuredValue, float maxTemp, float setPoint, int minSpeed, int maxSpeed){
  int motorSpeed;  
  if((messuredValue >= setPoint)&&(messuredValue <= maxTemp)){
    motorSpeed = map(messuredValue, setPoint, maxTemp, minSpeed, maxSpeed);
  }else if(messuredValue <= maxTemp){
    motorSpeed = 0;
  }else if(messuredValue >= setPoint){
    motorSpeed = 45;  
  }//end if
  
  return motorSpeed;
}//end speedControPLowTemp
*/
