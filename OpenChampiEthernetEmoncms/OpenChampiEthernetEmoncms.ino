/*
  Standart Gateway Trasmition Network v.2

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

//-----Need to be declared for correct Moteino functioning----
#include <Ethernet.h>
#include <utility/w5100.h> //https://github.com/kiwisincebirth/Arduino/tree/master/Ethernet
#include <avr/wdt.h>
#include <SoftwareSerial.h>
//------------------------------------------------------------


//---Debug Confiturations---
SoftwareSerial debug(2, 3);
#define dbg debug

#define SERIAL_BAUD   115200 
#define DEBUG //uncoment for debuging

//Emoncms APIKEY
#define APIKEY "d1699ac02ed979dd0c4af09b84a3c9f5"//Server 23.253.237.54

String msg = "";//Received packets

//---Ethernet Configurations---
byte mac[] = { 0x00, 0xCA, 0xEB, 0xC3, 0xDE, 0x03 };//  0xDE, 0xAD, 0xBE, 0xEF, 0xFE, 0xEE
IPAddress server(166, 78, 62, 254);//Numeric IP adress no DNS
byte ip[] = {192, 168, 10, 143};
byte gateway[] = {192, 168, 10, 1};
EthernetClient client;

//---Modbus Configurations---
#define RS485Transmit    HIGH
#define RS485Receive     LOW
#define SSerialTxControl 4   //RS485 Direction control
#define RS485Serial Serial
int byteReceived;
int byteSend;
unsigned char uchCRCHi = 0xFF; /* high byte of CRC initialized */
unsigned char uchCRCLo = 0xFF; /* low byte of CRC initialized */
byte id = 0x03;
int hex1, hex2;
byte piMsgBuff[20] = {0x03, 0x03, 15, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00};
String piMsgStr = "";
unsigned long piStartT;
unsigned long piIntervalT = 1000;

void halt(const __FlashStringHelper *error) {
  //Watchdog.enable(1000);
  wdt_enable(WDTO_1S);
  wdt_reset();
  dbg.println(error);
  while (1) {}//end while
}//end halt


void setup() {
  //---Ethernet Initial---
  W5100.select(7); //select pin 7 as SS for Ethernet module (KiwiSinceBirth mod)
  pinMode(8, OUTPUT);
  digitalWrite(8, HIGH);
  delay(500);
  //----------------------<

  
  dbg.begin(SERIAL_BAUD);
  RS485Serial.begin(19200);
  dbg.println(F("Serial Ready..."));

  pinMode(SSerialTxControl, OUTPUT);
  digitalWrite(SSerialTxControl, RS485Receive);  // Init Transceiver  
  
  piIntervalT *= 10;//every minute
  
  // start the Ethernet connection:
  if (Ethernet.begin(mac) == 0) {
    dbg.println("Failed to configure Ethernet using DHCP");
    // no point in carrying on, so do nothing forevermore:
    // try to congifure using IP address instead of DHCP:
    Ethernet.begin(mac, ip, dns, gateway);
  }else{
    dbg.println(F("IP ok"));  
  }
  // give the Ethernet shield a second to initialize:
  dbg.println(F("Ethernet Ready..."));
  dbg.println(Ethernet.localIP());
  delay(1000);  
  
}//end setup

void loop(){

  serialEvent();//Manage incomming packets
  
  if(Ethernet.maintain()%2){//if maintain() is 1 or 3 it has an error
    halt(F("No internet connection"));
  }//end if
  
  unsigned long piCurrentT = millis();
  if((piCurrentT - piStartT) > piIntervalT){
    piMsgStr = "";
    piStartT = piCurrentT;
  }//end if

  //Find CRC for modbus
  CRC16(piMsgBuff, 18);
  //Check if message is our
  if((piMsgBuff[0] == 0x03)&&(piMsgBuff[1] == 0x03)){//
    if((piMsgBuff[18] == uchCRCLo)&&(piMsgBuff[19] == uchCRCHi)){//Check valid message
      for(uint8_t i = 3; i< piMsgBuff[2]+3; i++){
		if(i < 16){
		  piMsgStr += piMsgBuff[i];
		  piMsgStr += " ";
		}else if(i == 16){
		  int value = hexToInt(piMsgBuff[i], piMsgBuff[i+1]);
		  piMsgStr += String(value);
		  break;
		}//end if
      }//end for
      dbg.println(piMsgStr);
      msg = piMsgStr;
      piMsgStr = "";
      for(uint8_t i = 0; i < 20; i++){
        piMsgBuff[i] = 0;
      }
    }//end if
  }//end if
 
  //msg = "3 54 65 23 54";//Message for degub purposes
  //delay(2000);
  if(msg != ""){//Check if msg is empty
    #if defined(DEBUG)
    	dbg.print(F("Packet received = "));
    #endif
	//---Sending to emoncms server---
    client.stop();
    if(client.connect(server,80)){
    #if defined(DEBUG)
    	  dbg.print(msg);//Print sensor values
    	  dbg.println();//end structure transmition
    	  dbg.println(F("Connencted"));
    #endif
	  client.print("GET /input/post.json?node=");  // make sure there is a [space] between GET and /input
    #if defined(DEBUG)
    	  dbg.print("GET /input/post.json?node=");
    #endif
	  uint8_t i = 0;
	  uint8_t node = -48;//Set cero
	  String temp_data = "";
	  for(i=0; i < msg.length(); i++){//Find node id
		if((msg[i] == ' ')){
		  client.print(node);
		  dbg.print(node);
		  temp_data = "";
		  i++;//Next position for csv data
		  break;
		}//end if
		node += int(msg[i]);
	  }//end for
	  node = 0;
	  client.print("&csv=");
    #if defined(DEBUG)
          dbg.print("&csv=");
    #endif
	  for(i; i < msg.length(); i++){
		if((msg[i] == ' ')){
		  temp_data += ',';
		}else{
		  temp_data += msg[i];
		}
	  }//end for
    
	  client.print(temp_data.c_str());
	  client.print("&apikey=");
	  client.print(APIKEY);         //assuming MYAPIKEY is a char or string
	  client.println(" HTTP/1.1");   //make sure there is a [space] BEFORE the HTTP
	  client.println(F("Host: 166.78.62.254"));
	  client.print(F("User-Agent: Arduino-ethernet"));
	  client.println(F("Connection: close"));
	  client.println();
	  //----------------------------------
#if defined(DEBUG)
      dbg.print(temp_data.c_str());
      dbg.print("&apikey=");
      dbg.print(APIKEY);         //assuming MYAPIKEY is a char or string
      dbg.println(" HTTP/1.1");   //make sure there is a [space] BEFORE the HTTP
      dbg.println(F("Host: 23.253.237.54"));
      dbg.print(F("User-Agent: Arduino-ethernet"));
      dbg.println(F("Connection: close"));
      dbg.println();
#endif
	  //----------------------------------
	}else{
	  dbg.println("Connection Failed");
    }//end if
  }//end if
  //Clean procedure
  msg = "";
  Serial.flush();
  dbg.flush();
}//end loop

void serialEvent(){
  uint8_t i = 0;
  while(RS485Serial.available()) { //Look for data from other Arduino
    delay(1);
    if (RS485Serial.available()){//Look for data from other Arduino
	  //      digitalWrite(Pin13LED, HIGH);// Show activity
      byteReceived = RS485Serial.read();// Read received byte
      piMsgBuff[i] = byteReceived;
	  //      digitalWrite(Pin13LED, LOW);// Show activity   
      byteReceived  = 0;
      i++;
    }//end if
  }//end while
}//end serialEvent


static unsigned char auchCRCHi[] = {
  0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0, 0x80, 0x41, 0x01, 0xC0, 0x80, 0x41, 0x00, 0xC1, 0x81, 0x40,
  0x01, 0xC0, 0x80, 0x41, 0x00, 0xC1, 0x81, 0x40, 0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0, 0x80, 0x41,
  0x01, 0xC0, 0x80, 0x41, 0x00, 0xC1, 0x81, 0x40, 0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0, 0x80, 0x41,
  0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0, 0x80, 0x41, 0x01, 0xC0, 0x80, 0x41, 0x00, 0xC1, 0x81, 0x40,
  0x01, 0xC0, 0x80, 0x41, 0x00, 0xC1, 0x81, 0x40, 0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0, 0x80, 0x41,
  0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0, 0x80, 0x41, 0x01, 0xC0, 0x80, 0x41, 0x00, 0xC1, 0x81, 0x40,
  0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0, 0x80, 0x41, 0x01, 0xC0, 0x80, 0x41, 0x00, 0xC1, 0x81, 0x40,
  0x01, 0xC0, 0x80, 0x41, 0x00, 0xC1, 0x81, 0x40, 0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0, 0x80, 0x41,
  0x01, 0xC0, 0x80, 0x41, 0x00, 0xC1, 0x81, 0x40, 0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0, 0x80, 0x41,
  0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0, 0x80, 0x41, 0x01, 0xC0, 0x80, 0x41, 0x00, 0xC1, 0x81, 0x40,
  0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0, 0x80, 0x41, 0x01, 0xC0, 0x80, 0x41, 0x00, 0xC1, 0x81, 0x40,
  0x01, 0xC0, 0x80, 0x41, 0x00, 0xC1, 0x81, 0x40, 0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0, 0x80, 0x41,
  0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0, 0x80, 0x41, 0x01, 0xC0, 0x80, 0x41, 0x00, 0xC1, 0x81, 0x40,
  0x01, 0xC0, 0x80, 0x41, 0x00, 0xC1, 0x81, 0x40, 0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0, 0x80, 0x41,
  0x01, 0xC0, 0x80, 0x41, 0x00, 0xC1, 0x81, 0x40, 0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0, 0x80, 0x41,
  0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0, 0x80, 0x41, 0x01, 0xC0, 0x80, 0x41, 0x00, 0xC1, 0x81, 0x40 };
/* Table of CRC values for low–order byte */

static char auchCRCLo[] = {
  0x00, 0xC0, 0xC1, 0x01, 0xC3, 0x03, 0x02, 0xC2, 0xC6, 0x06, 0x07, 0xC7, 0x05, 0xC5, 0xC4, 0x04,
  0xCC, 0x0C, 0x0D, 0xCD, 0x0F, 0xCF, 0xCE, 0x0E, 0x0A, 0xCA, 0xCB, 0x0B, 0xC9, 0x09, 0x08, 0xC8,
  0xD8, 0x18, 0x19, 0xD9, 0x1B, 0xDB, 0xDA, 0x1A, 0x1E, 0xDE, 0xDF, 0x1F, 0xDD, 0x1D, 0x1C, 0xDC,
  0x14, 0xD4, 0xD5, 0x15, 0xD7, 0x17, 0x16, 0xD6, 0xD2, 0x12, 0x13, 0xD3, 0x11, 0xD1, 0xD0, 0x10,
  0xF0, 0x30, 0x31, 0xF1, 0x33, 0xF3, 0xF2, 0x32, 0x36, 0xF6, 0xF7, 0x37, 0xF5, 0x35, 0x34, 0xF4,
  0x3C, 0xFC, 0xFD, 0x3D, 0xFF, 0x3F, 0x3E, 0xFE, 0xFA, 0x3A, 0x3B, 0xFB, 0x39, 0xF9, 0xF8, 0x38,
  0x28, 0xE8, 0xE9, 0x29, 0xEB, 0x2B, 0x2A, 0xEA, 0xEE, 0x2E, 0x2F, 0xEF, 0x2D, 0xED, 0xEC, 0x2C,
  0xE4, 0x24, 0x25, 0xE5, 0x27, 0xE7, 0xE6, 0x26, 0x22, 0xE2, 0xE3, 0x23, 0xE1, 0x21, 0x20, 0xE0,
  0xA0, 0x60, 0x61, 0xA1, 0x63, 0xA3, 0xA2, 0x62, 0x66, 0xA6, 0xA7, 0x67, 0xA5, 0x65, 0x64, 0xA4,
  0x6C, 0xAC, 0xAD, 0x6D, 0xAF, 0x6F, 0x6E, 0xAE, 0xAA, 0x6A, 0x6B, 0xAB, 0x69, 0xA9, 0xA8, 0x68,
  0x78, 0xB8, 0xB9, 0x79, 0xBB, 0x7B, 0x7A, 0xBA, 0xBE, 0x7E, 0x7F, 0xBF, 0x7D, 0xBD, 0xBC, 0x7C,
  0xB4, 0x74, 0x75, 0xB5, 0x77, 0xB7, 0xB6, 0x76, 0x72, 0xB2, 0xB3, 0x73, 0xB1, 0x71, 0x70, 0xB0,
  0x50, 0x90, 0x91, 0x51, 0x93, 0x53, 0x52, 0x92, 0x96, 0x56, 0x57, 0x97, 0x55, 0x95, 0x94, 0x54,
  0x9C, 0x5C, 0x5D, 0x9D, 0x5F, 0x9F, 0x9E, 0x5E, 0x5A, 0x9A, 0x9B, 0x5B, 0x99, 0x59, 0x58, 0x98,
  0x88, 0x48, 0x49, 0x89, 0x4B, 0x8B, 0x8A, 0x4A, 0x4E, 0x8E, 0x8F, 0x4F, 0x8D, 0x4D, 0x4C, 0x8C,
  0x44, 0x84, 0x85, 0x45, 0x87, 0x47, 0x46, 0x86, 0x82, 0x42, 0x43, 0x83, 0x41, 0x81, 0x80, 0x40 };
/* The function returns the CRC as a unsigned short type */
//  unsigned char *puchMsg; /* message to calculate CRC upon */
//unsigned short usDataLen; /* quantity of bytes in message */
unsigned short CRC16(unsigned char *puchMsg, unsigned short usDataLen){
  //unsigned char uchCRCHi = 0xFF; /* high byte of CRC initialized */
  //unsigned char uchCRCLo = 0xFF; /* low byte of CRC initialized */
  uchCRCHi = 0xFF;
  uchCRCLo = 0xFF;
 
  unsigned uIndex; /* will index into CRC lookup table */
  while (usDataLen--) /* pass through message buffer */
	{
	  uIndex = uchCRCLo ^ *puchMsg++; /* calculate the CRC */
	  uchCRCLo = uchCRCHi ^ auchCRCHi[uIndex];
	  uchCRCHi = auchCRCLo[uIndex];
	}
  //dbg.println(uchCRCLo, HEX);
  //dbg.println(uchCRCHi, HEX);
  return (uchCRCHi << 8 | uchCRCLo);
}//end CRC16

int hexToInt(int spdhex1, int spdhex2){
  String hexadecimal = "";
  char vals[4] = {'0','0','0','0'};
  int vhex = 0;
  if(String(spdhex1, HEX).length() < 2){
    hexadecimal += "0";
    hexadecimal += String(spdhex1, HEX);
  }else{
    hexadecimal = String(spdhex1, HEX);
  }//end if
  if(String(spdhex2, HEX).length() < 2){
    hexadecimal += "0";
    hexadecimal += String(spdhex2, HEX);    
  }else{
    hexadecimal += String(spdhex2, HEX);
  }//end if

  uint8_t j = 0;
  uint8_t i = 0;
  for(i = 3; i > 0; i--){
    vals[i] = (hexadecimal[(hexadecimal.length()-1)-j]);
    j++;
  }//end for
  char const hex_chars[16] = { '0', '1', '2', '3', '4', '5', '6', '7', '8', '9', 'a', 'b', 'c', 'd', 'e', 'f' };  
  for(i = 0; i < 4; i++){
    for(j = 0; j < 16; j++){
      if(vals[i] == hex_chars[j]){
        vhex += round((j*pow(16, 3-i)));//
        break;
      }//end if
    }//end for
  }//end for
  //dbg.print("value = ");
  //dbg.println(vhex);
  return vhex;
}
