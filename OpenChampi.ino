/*
  OpenChampi
  Automatization control for a mushroom farm

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

#include "Mollier.h"
#include "UmaAction.h"
#include "RoomControl.h"
#include "WegVFD.h"
#include <OneWire.h>
#include <DallasTemperature.h>
#include <MQ135.h> //https://github.com/GeorgK/MQ135
#include <Nextion.h>
#include <EEPROM.h>

#define nextion Serial2
//#define DEBUG
//#define DEBUG2
#define MINPRESSURE 10
#define MAXPRESSURE 1000
#define ONE_WIRE_BUS_1 5
#define ONE_WIRE_BUS_2 6
#define ONE_WIRE_BUS_3 7
//#define ONE_WIRE_BUS_4 8
#define CO2SENSOR A0
#define SSerialTxControl 3   //RS485 Direction control
#define RS485Serial Serial1
#define RS485Transmit    HIGH
#define RS485Receive     LOW
#define pin13Led         13


//======Screen Button Definitions

#define CONTROL_INCUB_1 "65 1 6 1 ffff ffff ffff"
#define CONTROL_INCUB_2 "65 1 7 1 ffff ffff ffff"
#define CONTROL_INDUCTION "65 1 8 1 ffff ffff ffff"
#define CONTROL_GROWTH_1 "65 1 9 1 ffff ffff ffff"
#define CONTROL_GROWTH_2 "65 1 a 1 ffff ffff ffff"
#define CONTROL_SECADO "65 1 b 1 ffff ffff ffff"
#define RELAY_COLD_WATER "65 2 6 1 ffff ffff ffff"
#define RELAY_HOT_WATER "65 2 7 1 ffff ffff ffff"
#define RELAY_DAMPER "65 2 8 1 ffff ffff ffff"
#define RELAY_ALARM "65 2 9 1 ffff ffff ffff"
#define LIMITS_LOW_HP "65 3 c 1 ffff ffff ffff"
#define LIMITS_LOW_HL "65 3 d 1 ffff ffff ffff"
#define LIMITS_HIGH_HP "65 3 e 1 ffff ffff ffff"
#define LIMITS_HIGH_HL "65 3 f 1 ffff ffff ffff"
#define LIMITS_LOW_TP "65 3 10 1 ffff ffff ffff"
#define LIMITS_LOW_TL "65 3 11 1 ffff ffff ffff"
#define LIMITS_HIGH_TP "65 3 12 1 ffff ffff ffff"
#define LIMITS_HIGH_TL "65 3 13 1 ffff ffff ffff"
#define LIMITS_LOW_CO2P "65 3 14 1 ffff ffff ffff"
#define LIMITS_LOW_CO2L "65 3 19 1 ffff ffff ffff"
#define LIMITS_HIGH_CO2P "65 3 1a 1 ffff ffff ffff"
#define LIMITS_HIGH_CO2L "65 3 1b 1 ffff ffff ffff"
#define MODE_AUTO "65 5 8 1 ffff ffff ffff"
#define MODE_MANUAL "65 5 7 1 ffff ffff ffff"

uint8_t i = 0;

//---Structures declarations---

struct relaysModule{
  uint8_t aguaFriaOn = 38;//Open   
  uint8_t aguaFriaOff = 39;//Close
  uint8_t aguaCalienteOn = 40;//Open
  uint8_t aguaCalienteOff = 41;//Close
  uint8_t damper = 42; //ON
  uint8_t pumpCO2 = 43; //ON
  uint8_t alarm = 44; //ON
  //uint8_t resetVfd = 45; //ON?????
}relays;

//---Class declarations---
Mollier *calculus;//Created in heap
MQ135 gasSensor(CO2SENSOR);
//---temperatureChecker---
double *compostSensors;
double *mollierSensors;
//---Weg VFD---
//WegVFD *cfw500;
WegVFD cfw500(RS485Serial, 19200, SSerialTxControl);
//---Nextion Screen---
Nextion myNextion(nextion, 9600); //create a Nextion object named myNextion using the nextion serial port @ 9600bps

//----Structure definition in Mollier class---
//setPointVariables spHumedad;


//float M, relativeHumidity, absouleHumidity, specificHumidity, outdoorHumidity, absoluteHumidityOutside;//variable de HR 

//----------------State Machines Variables-------------------
//actionRelayStartT,
unsigned long co2PumpStartT, 
              co2SensorStartT, 
              modStartT, 
              chModStart,
              piModStartT, 
              checkerStartT, 
              page0StartT, 
              page1StartT, 
              page2StartT, 
              page3StartT,
              page5StartT;
//actionRelayIntervalT = 1000,
  
unsigned long co2PumpIntervalT = 1000,
  checkerIntervalT = 1000,//1s default
  modIntervalT = 1000,
  chModInterval = 1000,
  piModIntervalT = 1000,
  page0IntervalT = 1000,
  page1IntervalT = 1000,
  page2IntervalT = 1000,
  page3IntervalT = 1000,
  page5IntervalT = 1000;
//-----------------------------------------------------------

//---Drying the room, manually set---
boolean goToDryTime = true;
unsigned long dryingStartT, dryingTotalT;

//---CO2 Level in the room---
float co2Level = 0;

double averageTemperature = 0;
float externalTemperature;//Standard
float externalHumidity;//Just a # for debug

//---RoomControl---
uint8_t controlState = 0;
String systemPhase = "incubacion1";
String manualRelay = "";
//=====================================================================

//---Motor Speed Control---
int byteReceived, byteSend, spdhex1, spdhex2, motorSpeed;
unsigned char uchCRCHi = 0xFF; /* high byte of CRC initialized */
unsigned char uchCRCLo = 0xFF; /* low byte of CRC initialized */
byte buff[8] = {0x01, 0x06, 0x02, 0xAB, 0x00, 0x00, 0xF9, 0x92};//Default 0Hz 
int mSpeed = 0;

//---Chiller Modbus---
byte chillerBuff[7] = {0x01, 0x06, 0x00, 0x00, 0x00, 0xAB, 0x05};
//ID, MODIF, ROOM, CHILLER_STATE, BOILER_STATE, CRC-, CRC+
byte chillerAnswer[6] = {0x02, 0x06, 0x00, 0x00, 0x00, 0x00};
//ID, MODIF, TEMP, HUMID, CRC-, CRC+

//---RaspberryPi Modbus---
byte piMsgBuff[31] = {0x03, 0x03, 26, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00};
//ID, READ REGISTERS, COUNT, LoTEMP, HiTEMP, LoEXTtemp, HiEXTtemp, LoEXThum, HiEXThum, LoAMBtEMP, HiAMBtEMP, LoPVA, HiPVA, LoPVS, HiPVS, LoDVT, HiDVT, LoHR, HiHR, LoHA, HiHA, LoDEW, HiDEW, LoDVA, HiDVA, LoHE, HiHE, LoCO2, HiCO2, CRC-, CRC+
//ID, READ REGISTERS, COUNT, TEMP, EXTtemp, EXThum, AMBtEMP, PVA, PVS, DVT, HR, HA, DEW, DVA, HE, CO2

//---Screen---
char pageId[6] = {0x30, 0x31, 0x32, 0x33, 0x34, 0x35};
uint8_t actualPage = 0;
String actualState = "";
String pastState = "";
boolean coldWater = true, 
  hotWater = true,
  damper = true,
  alarm = true;//Manual relays state
String incommingMsg, icomBtn;

//---Limits--
uint8_t Ltemp, Htemp; 
float Lco2, Hco2, Lhumidity, Hhumidity;

void setup() {

Serial.begin(19200);
Serial.println("Starting OpenChampi Automatization Control...");
Serial.flush();


  //Set all relay pins in a few rows posible as outputs
  for(i = 38; i < 46; i++){//8 relays
    pinMode(i,OUTPUT);//Pin Mode
    digitalWrite(i, LOW);//Default state LOW
  }//end for

  for(i = 6; i < 14; i++){
    pinMode(i, INPUT_PULLUP);
  }//end for
  myNextion.init();
  compostSensors = new double[8];
  mollierSensors = new double[2];
  calculus = new Mollier(ONE_WIRE_BUS_1, ONE_WIRE_BUS_2, ONE_WIRE_BUS_3);
  //gasSensor = new MQ135(CO2SENSOR);
  
  //actionRelayIntervalT *= 45;//45 seconds
  co2PumpIntervalT *= 60*30;//Every 30 minutes
  checkerIntervalT *= 60*0.25;//Every 5 minutes

  modIntervalT *= 3;//Weg interval
  chModInterval *= 7;//Chiller interval
  piModIntervalT *= 10;//Raspberry Pi interval

  page0IntervalT *= 4;//Every 4 seconds
  page1IntervalT *= 2;
  page2IntervalT *= 2;
  page3IntervalT *= 3;//Every 3 seconds
  page5IntervalT *= 2;

  //cfw500 = new WegVFD(RS485Serial, 19200, SSerialTxControl);
  pinMode(SSerialTxControl, OUTPUT);
  digitalWrite(SSerialTxControl, RS485Receive);  // Init Transceiver    
  eepromBack();
}//setup

void loop() {
  serialEvent1();
  serialEvent2();
  if(actualPage == 0){
    unsigned long  page0CurrentT = millis();
    if(page0CurrentT - page0StartT > page0IntervalT){
      updatePage0();
      page0StartT = page0CurrentT;
    }//end if
  }else if(actualPage == 1){//
    if(actualState != pastState){
      actualState = pastState;
      myNextion.setComponentText("t6", actualState);//String(actualState)
    }//end if
  }//end if
  
  unsigned long globalCurrentTime = 0;
  
  //automatic update temperatures readings and Moliere calculus
  globalCurrentTime = millis();
  temperatureChecker(globalCurrentTime);

  //automatic update co2 readings
  globalCurrentTime = millis();//Update current time
  co2Pump(globalCurrentTime);


  if(icomBtn  == MODE_AUTO){
    controlState = 0;
  }else if(icomBtn == MODE_MANUAL){
    controlState = 1;
  }else if(icomBtn == CONTROL_SECADO){
    controlState = 2;
  }

  if((icomBtn == RELAY_ALARM)&&(controlState != 1)){//Alarma always manual mode
    if(alarm){
      digitalWrite(relays.alarm, HIGH);
      myNextion.setComponentText("b3", "Alarm ON");
      alarm = false;
	  }else{
      digitalWrite(relays.alarm, LOW);
      myNextion.setComponentText("b3", "Alarm OFF");
      alarm = true;
	  }//end if    
	  icomBtn = "";
  }//end if


  if((icomBtn == CONTROL_SECADO)&&(controlState == 2)){//Secado
    updateSecado();
  }else if((icomBtn != CONTROL_SECADO)&&(controlState == 2)){
    controlState = 0;
  }//end if

  if(controlState == 0){//Automatic
    updatePage1();
  }else if((actualPage == 2)&&(controlState == 1)){//Manual
    updatePage2();
  }

  if((actualPage == 1)&&(controlState != 1)){
    globalCurrentTime = millis();//Update current time
    if(globalCurrentTime - page1StartT > page1IntervalT){
      myNextion.setComponentText("t6", pastState.c_str());
      page1StartT = globalCurrentTime;
    }//end if
  }//end if

  if((actualPage == 2)){
    globalCurrentTime = millis();//Update current time
    if(globalCurrentTime - page2StartT > page2IntervalT){    
      if(digitalRead(relays.alarm)){
        myNextion.setComponentText("b3", "Alarm ON");  
      }else{
        myNextion.setComponentText("b3", "Alarm OFF");  
      }//end if
      page2StartT = globalCurrentTime;
    }//end if
  }//end if
  
  if(actualPage == 3){
    limits();
    globalCurrentTime = millis();//Update current time
    if(globalCurrentTime- page3StartT > page3IntervalT){
      updateLimits();
      page3StartT = globalCurrentTime;
    }//end if
  }//end if

  if((actualPage == 5)&&(controlState == 1)){//Manual
    globalCurrentTime = millis();//Update current time
    if(globalCurrentTime- page5StartT > page5IntervalT){
      motorSpeed = myNextion.getComponentValue("h0");
      myNextion.setComponentText("t6", String(motorSpeed).c_str());
      page5StartT = globalCurrentTime;
    }//enf if
  }

  globalCurrentTime = millis();//Update current time
  if((globalCurrentTime - modStartT) > modIntervalT){
    modBusSpeed(averageTemperature, motorSpeed);
    modStartT = globalCurrentTime;
  }//end if
  globalCurrentTime = millis();//Update current time
  if((globalCurrentTime - chModStart) > chModInterval){
      #if defined(DEBU2)
        for(i = 0; i < 7; i++){
          Serial.print(chillerBuff[i]);
          Serial.print("-");
        }//end if
        Serial.println();
      #endif
    modBusChiller(2, 1, digitalRead(relays.aguaFriaOn), digitalRead(relays.aguaCalienteOn));
    chModStart = globalCurrentTime;
  }//end if

  globalCurrentTime = millis();//Update current time
  if((globalCurrentTime - piModStartT) > piModIntervalT){
    //Serial.println("Sendig Report to Raspberry Pi");
    modBusPi(3);
    piModStartT = globalCurrentTime;
  }//end if

  
}//end loop

#if defined(DEBUG)
  int freeRam (){
    extern int __heap_start, *__brkval; 
    int v; 
    return (int) &v - (__brkval == 0 ? (int) &__heap_start : (int) __brkval); 
  }
#endif

int co2Pump(unsigned long co2PumpCurrenTime){
  //Start suck out 45s before read CO2 level
  const unsigned int timeBefore = 1000*45;
  //  Serial.println(timeBefore);
  if((co2PumpCurrenTime - co2PumpStartT) > (co2PumpIntervalT-timeBefore)){
#if defined(DEBUG)
	Serial.println("CO2 suction start!!!");
#endif
    digitalWrite(relays.pumpCO2, HIGH);
    co2PumpStartT = co2PumpCurrenTime;
  }
  co2PumpCurrenTime = millis();
  if(co2PumpCurrenTime - co2SensorStartT > co2PumpIntervalT){
#if defined(DEBUG)
	Serial.println("CO2 suction stop!!!");
	Serial.println("Reading MQ135 Sensor");
#endif
    digitalWrite(relays.pumpCO2, LOW);
    co2Level = gasSensor.getPPM();//
#if defined(DEBUG)
	Serial.println("CO2 Level = " + String(co2Level));
#endif
    co2SensorStartT = co2PumpCurrenTime;
    co2PumpStartT = co2PumpCurrenTime;
  }//end if
  return 1;
}//end co2Pump

int temperatureChecker(unsigned long checkerCurrentTime){
  if(checkerCurrentTime - checkerStartT > checkerIntervalT){
#if defined(DEBUG)
	Serial.println("Reading temperature Sensors");
#endif
    calculus->readSensorTemperatures();
    
    calculus->getCompostTemperatureSensors(compostSensors);
    calculus->getMollierTemperatureSensors(mollierSensors);
#if defined(DEBUG)    
	for(uint8_t i = 0; i<8; i++){
	  if(i < 6){
		Serial.print("Compost Sensor " + String(i+1) + " = ");
		Serial.println(compostSensors[i]);
	  }
	}//end for
	for(uint8_t i = 0; i<2; i++){
	  Serial.print("Moliere Sensor " + String(i+1) + " = ");
	  Serial.println(mollierSensors[i]);      
	}//end for
#endif

    //---Temperature average in compost sensors---
    double tempTemp = calculus->compostSensorsAverage();
    if(tempTemp > 0){
      averageTemperature = tempTemp;
    }else{
	  //alarm  
    }
    
#if defined(DEBUG)
	Serial.println("Compost average temperature  = " + String(averageTemperature));
      
	//---Getting Mollier calculus for extern and intern variables----
	Serial.println("Getting Moliere calculus");
#endif
    calculus->mollierCalculus();
#if defined(DEBUG)
	Serial.print("Internal Moliere = ");
	Serial.println(calculus->mollierData.HR);
#endif
    checkerStartT = checkerCurrentTime;
  }//end if

  if((chillerAnswer[0] == 0x02)&&(chillerAnswer[1] == 0x06)){
    cfw500.CRC16(chillerAnswer, 4, uchCRCLo, uchCRCHi);
    if((chillerAnswer[4] == uchCRCLo)&&(chillerAnswer[5] == uchCRCHi)){
      externalTemperature =  chillerAnswer[2];
      externalHumidity = chillerAnswer[3];
    }//end if
    chillerAnswer[0] = 0;
  }//end if

  
  return 1;
}//end temperatureChecker

void serialEvent1(){
  i = 0;
  while(RS485Serial.available()) { //Look for data from cfw500
    delay(1);
    if (RS485Serial.available()){  //Look for data from other cfw500
      digitalWrite(pin13Led, HIGH);  // Show activity
      byteReceived = RS485Serial.read();// Read received byte
      chillerAnswer[i] = byteReceived;
      i++;
      //Serial.print(byteReceived, HEX);// Show on Serial Monitor
      //Serial.print("-");
      //digitalWrite(pin13Led, LOW);  // Show activity   
      byteReceived  = 0;
    }//end if
  }//end while
}//end serialEvent1


void serialEvent2(){
  incommingMsg = myNextion.listen(); //check for message
  if((incommingMsg != "")&&(incommingMsg.length() == 1)){ // if a message is received...
    char pageId[6] = {0x30, 0x31, 0x32, 0x33, 0x34, 0x35};
    for(i = 0; i< 6; i++){
      if(pageId[i] == incommingMsg[0]){
        actualPage = i;
        EEPROM.write(0, i);//Save page
//        #if defined(DEBU2)
          Serial.print("msg = ");
          Serial.print(incommingMsg);
          Serial.print(", pagina = ");
          Serial.println(actualPage); 
//        #endif
        break;
      }//end if
    }//end for
  }//end if
  else if(incommingMsg != ""){
    icomBtn = incommingMsg;
    #if defined(DEBUG)
      Serial.print("Boton = ");
      Serial.println(incommingMsg);
    #endif
    if(actualPage == 1){
      systemPhase = incommingMsg;
    }else if(actualPage == 2){
      manualRelay = incommingMsg;
    }else if(actualPage == 5){
      //mode = incommingMsg;
    }//end if
  }//end if
}//end serialEvent2

void modBusPi(byte id){
//ID, READ REGISTERS, COUNT, LoTEMP, HiTEMP, LoEXTtemp, HiEXTtemp, LoEXThum, HiEXThum, LoAMBtEMP, HiAMBtEMP, LoPVA, HiPVA, LoPVS, HiPVS, LoDVT, HiDVT, LoHR, HiHR, LoHA, HiHA, LoDEW, HiDEW, LoDVA, HiDVA, LoHE, HiHE, LoCO2, HiCO2, CRC-, CRC+
//cfw500->twoHex(speed, spdhex1, spdhex2);
  spdhex1 = spdhex2 = 0;

  uint8_t j = 0;
  //piMsgBuff[0] = id;
  cfw500.twoHex(averageTemperature, spdhex1, spdhex2);
  piMsgBuff[3] = spdhex1;
  piMsgBuff[4] = spdhex2;
  spdhex1 = spdhex2 = 0;
  cfw500.twoHex(externalTemperature, spdhex1, spdhex2);
  piMsgBuff[5] = spdhex1;
  piMsgBuff[6] = spdhex2;
  spdhex1 = spdhex2 = 0;
  cfw500.twoHex(externalHumidity, spdhex1, spdhex2);
  piMsgBuff[7] = spdhex1;
  piMsgBuff[8] = spdhex2;
  spdhex1 = spdhex2 = 0;
  cfw500.twoHex(mollierSensors[0], spdhex1, spdhex2);
  piMsgBuff[9] = spdhex1;
  piMsgBuff[10] = spdhex2;
  spdhex1 = spdhex2 = 0;
  cfw500.twoHex(calculus->mollierData.pva, spdhex1, spdhex2);
  piMsgBuff[11] = spdhex1;
  piMsgBuff[12] = spdhex2;
  spdhex1 = spdhex2 = 0;
  cfw500.twoHex(calculus->mollierData.pvs, spdhex1, spdhex2);
  piMsgBuff[13] = spdhex1;
  piMsgBuff[14] = spdhex2;
  spdhex1 = spdhex2 = 0;
  cfw500.twoHex(calculus->mollierData.dvt, spdhex1, spdhex2);
  piMsgBuff[15] = spdhex1;
  piMsgBuff[16] = spdhex2;
  spdhex1 = spdhex2 = 0;
  cfw500.twoHex(calculus->mollierData.HR, spdhex1, spdhex2);
  piMsgBuff[17] = spdhex1;
  piMsgBuff[18] = spdhex2;
  spdhex1 = spdhex2 = 0;
  cfw500.twoHex(calculus->mollierData.HA, spdhex1, spdhex2);
  piMsgBuff[19] = spdhex1;
  piMsgBuff[20] = spdhex2;
  spdhex1 = spdhex2 = 0;
  cfw500.twoHex(calculus->mollierData.DEW, spdhex1, spdhex2);
  piMsgBuff[21] = spdhex1;
  piMsgBuff[22] = spdhex2;
  spdhex1 = spdhex2 = 0;
  cfw500.twoHex(calculus->mollierData.DVA, spdhex1, spdhex2);
  piMsgBuff[23] = spdhex1;
  piMsgBuff[24] = spdhex2;
  spdhex1 = spdhex2 = 0;
  cfw500.twoHex(calculus->mollierData.HE, spdhex1, spdhex2);
  piMsgBuff[25] = spdhex1;
  piMsgBuff[26] = spdhex2;
  spdhex1 = spdhex2 = 0;
  cfw500.twoHex(co2Level, spdhex1, spdhex2);
  piMsgBuff[27] = spdhex1;
  piMsgBuff[28] = spdhex2;
  spdhex1 = spdhex2 = 0;
  cfw500.CRC16(piMsgBuff, 28, uchCRCLo, uchCRCHi);
  piMsgBuff[29] = uchCRCLo;
  piMsgBuff[30] = uchCRCHi;
//  for(i = 0; i < 31; i++){
//    Serial.print(piMsgBuff[i]);  
//    Serial.print("-");
//  }
//  Serial.println();
  
  digitalWrite(pin13Led, HIGH);  // Show activity
  cfw500.enableTransmit();
  delay(11);
  RS485Serial.write(piMsgBuff, 31);
  delay(21); 
  cfw500.disableTransmit();
  digitalWrite(pin13Led, LOW);  // Show activity

}//end modBusPi

void modBusChiller(byte id, byte idRoom, byte cWaterOn, byte hWaterOn){
 //ID, MODIF, ROOM, CHILLER_STATE, BOILER_STATE, CRC-, CRC+
 chillerBuff[0] = id;
 chillerBuff[2] = idRoom;
 chillerBuff[3] = cWaterOn;
 chillerBuff[4] = hWaterOn;
  digitalWrite(pin13Led, HIGH);  // Show activity

  cfw500.CRC16(chillerBuff, 5, uchCRCLo, uchCRCHi);//Find CRC+ and CRC-
  chillerBuff[5] = uchCRCLo;
  chillerBuff[6] = uchCRCHi;  
  //  Serial.println("Sending message...");  
  cfw500.enableTransmit();

  delay(11);
  RS485Serial.write(chillerBuff, 7);
  delay(11);
  
  cfw500.disableTransmit();
  digitalWrite(pin13Led, LOW);  // Show activity
}//end modBusChiller

void modBusSpeed(float mValue, uint8_t speed){
    //Serial.print("Temp = ");
    //Serial.print(mValue);
    //Serial.print("--MSpeed = ");
    //Serial.println(speed);
  cfw500.speedtwoHex(speed, spdhex1, spdhex2);
  buff[4] = spdhex1;
  buff[5] = spdhex2;
 
  digitalWrite(pin13Led, HIGH);  // Show activity

  cfw500.CRC16(buff, 6, uchCRCLo, uchCRCHi);//Find CRC+ and CRC-
  buff[6] = uchCRCLo;
  buff[7] = uchCRCHi;  
  //  Serial.println("Sending message...");  
  cfw500.enableTransmit();

  delay(5);
  RS485Serial.write(buff, 8);
  delay(5);
  
  cfw500.disableTransmit();

  digitalWrite(pin13Led, LOW);  // Show activity 

  //while-----...
#if defined(DEBUG) 
  Serial.println("");
#endif
  spdhex1 = spdhex2 = 0;

  //cfw500->setFrecuency(speed);
  
//  delay(500);//Origin 1000

}//end modBusSpeed

void updatePage0(void){
  myNextion.setComponentText("t10", String(averageTemperature));delay(25);
  myNextion.setComponentText("t11", String(externalTemperature));delay(25);
  myNextion.setComponentText("t12", String(externalHumidity));delay(25);
  myNextion.setComponentText("t13", String(mollierSensors[0]));delay(25);
  myNextion.setComponentText("t14", String(calculus->mollierData.HR));delay(25);
  myNextion.setComponentText("t20", String(calculus->mollierData.pva));delay(25);
  myNextion.setComponentText("t21", String(calculus->mollierData.pvs));delay(25);
  myNextion.setComponentText("t22", String(calculus->mollierData.dvt));delay(25);
  myNextion.setComponentText("t23", String(calculus->mollierData.HR));delay(25);
  myNextion.setComponentText("t24", String(calculus->mollierData.HA));delay(25);
  myNextion.setComponentText("t32", String(calculus->mollierData.DEW));delay(25);
  myNextion.setComponentText("t31", String(calculus->mollierData.DVA));delay(25);
  myNextion.setComponentText("t30", String(calculus->mollierData.HE));delay(25);
  myNextion.setComponentText("t29", String(co2Level));delay(25);
}//end updatePage0

void updatePage1(void){
  //--------Room Control----------
  //TODO seach type of control
  //controlState = //Take from screen???
  /*In automatic control the user can change limit parameters
	but user can't act directly on the system
	In manual control the user have total control over the system,
	no parameter is controlled automatically */  
  //systemPhase = //Take form screen, from EEPROM or both???   

/*
uint8_t Ltemp, Htemp; 
float Lco2, Hco2, Lhumidity, Hhumidity;
*/
  
  if(systemPhase == CONTROL_INCUB_1){
    if(controlState == 0){
      Ltemp = 24;
      Htemp = 27;
    }//end if
	  motorSpeed = incubationPhase1(averageTemperature, Ltemp, Htemp, mollierSensors[0], externalTemperature);
	  //Serial.println("INCUBACION 1");
	  pastState = "incubacion 1";
  }else if(systemPhase == CONTROL_INCUB_2){
    if(controlState == 0){
      Ltemp = 25;
      Htemp = 27;
      Lco2 = 3500;
      Hco2 = 10000;
      Lhumidity = 99;
      Hhumidity = 100;
    }//end if    
	  motorSpeed =  incubationPhase2(averageTemperature, Ltemp, Htemp, externalTemperature, mollierSensors[0], co2Level, Lco2, Hco2, externalHumidity, calculus->mollierData.HR, Lhumidity, Hhumidity, 24);
	  //Serial.println("INCUBACION 2");
	  pastState = "incubacion 2";
  }else if(systemPhase == CONTROL_INDUCTION){
    if(controlState == 0){
      Ltemp = 19;
      Htemp = 20;
      Lco2 = 1200;
      Hco2 = 3500;
      Lhumidity = 88;
      Hhumidity = 91;
    }//end if       
	  motorSpeed =  induction(averageTemperature, Ltemp, Htemp, externalTemperature, mollierSensors[0], co2Level, Lco2, Hco2, externalHumidity, calculus->mollierData.HR, Lhumidity, Hhumidity, 24);
	  //Serial.println("INDUCCION");
	  pastState = "induccion";
  }else if(systemPhase == CONTROL_GROWTH_1){
    if(controlState == 0){
      Ltemp = 19;
      Htemp = 20;
      Lco2 = 1200;
      Hco2 = 1600;
      Lhumidity = 87;
      Hhumidity = 90;
    }//end if     
	  mSpeed = startGrowth(averageTemperature, Ltemp, Htemp, externalTemperature, mollierSensors[0], co2Level, Lco2, Hco2, externalHumidity, calculus->mollierData.HR, Lhumidity, Hhumidity, 24);
	  //Serial.println("CRECIMIENTO 1");
	  pastState = "crecimiento 1";
  }else if(systemPhase == CONTROL_GROWTH_2){
    if(controlState == 0){
      Ltemp = 19;
      Htemp = 20;
      Lhumidity = 87;
      Hhumidity = 89;
    }//end if       
	  motorSpeed = growth(averageTemperature, Ltemp, Htemp, externalTemperature, mollierSensors[0], externalHumidity, calculus->mollierData.HR, Lhumidity, Hhumidity, 24);    
	  //Serial.println("CRECIMIENTO 2");
	  pastState = "crecimiento 2";
  }else if(systemPhase == CONTROL_SECADO){
	  //Serial.println("SECADO");
	  pastState = "secado";
	  goToDryTime = true;
	  controlState = 2;
  }//end if

}//end updatePage1

void updatePage2(void){
  if(manualRelay == RELAY_COLD_WATER){//systemPhase == CONTROL_INCUB_1
    if(coldWater){
      //Serial.println("Agua fria On");
      digitalWrite(relays.aguaFriaOn, HIGH);
      digitalWrite(relays.aguaFriaOff, LOW);
      myNextion.setComponentText("b0", "Agua Fria ON");
      coldWater = false;
    }else{
      //Serial.println("Agua fria off");
      digitalWrite(relays.aguaFriaOn, LOW);
      digitalWrite(relays.aguaFriaOff, HIGH);
      myNextion.setComponentText("b0", "Agua Fria OFF");
      coldWater = true;
    }//end if
  }else if(manualRelay == RELAY_HOT_WATER){
    if(hotWater){
      //Serial.println("Agua Caliente On");
      digitalWrite(relays.aguaCalienteOn, HIGH);
      digitalWrite(relays.aguaCalienteOff, LOW);
      myNextion.setComponentText("b1", "Agua Caliente ON");
      hotWater = false;
    }else{
      //Serial.println("Agua Caliente off");
      digitalWrite(relays.aguaCalienteOn, LOW);
      digitalWrite(relays.aguaCalienteOff, HIGH);
      myNextion.setComponentText("b1", "Agua Caliente OFF");
      hotWater = true;
    }//end if
  }else if(manualRelay == RELAY_DAMPER){
    if(damper){
      digitalWrite(relays.damper, HIGH);
      myNextion.setComponentText("b2", "Damper ON");
      damper = false;
    }else{
      digitalWrite(relays.damper, LOW);
      myNextion.setComponentText("b2", "Damper OFF");
      damper = true;
    }//end if
  }else if(manualRelay == RELAY_ALARM){
    if(alarm){
      digitalWrite(relays.alarm, HIGH);
      myNextion.setComponentText("b3", "Alarm ON");
      alarm = false;
    }else{
      digitalWrite(relays.alarm, LOW);
      myNextion.setComponentText("b3", "Alarm OFF");
      alarm = true;
    }//end if
  }//endif
  manualRelay = "";
}//end updatePage2

void updateSecado(void){
  if(goToDryTime){
  	drying();//----Start drying
  	dryingStartT = millis();
  	goToDryTime = false;
  	myNextion.setComponentText("t6", "secado");
  }
  unsigned long dryingStopT = millis();
  dryingTotalT = (dryingStartT-dryingStopT)*1000*60;//Time in minutes
  //Serial.println(dryingTotalT);
  //si humedad <= 88% cambiar a caso 1
  if(calculus->mollierData.HR <= 0.88){
  	controlState = 0;
  	goToDryTime = true;
  }//end if 
}//end updateSecado

void eepromBack(){
  actualPage = EEPROM.read(0);
  String page = "page " + String(actualPage);
  myNextion.sendCommand(page.c_str());

  //if(Lhumidity != 0)
  //  Lhumidity = EEPROM.read(1);
/*	if(Hhumidity != 0)
    Hhumidity = EEPROM.read(2);
	if(Ltemp != 0)
    Ltemp = EEPROM.read(4);
	if(Htemp != 0)
    Htemp = EEPROM.read(6);
	if(Hco2 != 0)
    Hco2 = EEPROM.read(8);
	if(Lco2 != 0)
    Lco2 = EEPROM.read(10);*/
}//end eempromBack

void limits(void){
  //uint8_t Ltemp, Htemp, Lco2, Hco2, Lhumidity, Hhumidity;
  if(icomBtn == LIMITS_LOW_HP){
    if(Hhumidity > Lhumidity){
      Lhumidity++;
    }
    myNextion.setComponentText("t7", String(Lhumidity));
	  //EEPROM.write(1, Lhumidity);
  }else if(icomBtn == LIMITS_LOW_HL){
    if(Lhumidity > 0){
      Lhumidity--;  
    }
    myNextion.setComponentText("t7", String(Lhumidity));
	  //EEPROM.write(1, Lhumidity);
  }else if(icomBtn == LIMITS_HIGH_HP){
    if(Lhumidity <= Hhumidity){
      Hhumidity++;
    }
    myNextion.setComponentText("t8", String(Hhumidity));
//	  EEPROM.write(2, Hhumidity);
  }else if(icomBtn == LIMITS_HIGH_HL){
    if((Hhumidity > 0)&&(Hhumidity >= Lhumidity)){
      Hhumidity--;
    }//end if
    myNextion.setComponentText("t8", String(Hhumidity));
//	  EEPROM.write(2, Hhumidity);
  }else if(icomBtn == LIMITS_LOW_TP){
    if(Htemp > Ltemp){
    Ltemp++;
    }//end if
    myNextion.setComponentText("t9", String(Ltemp));
//	  EEPROM.write(4, Ltemp);
  }else if(icomBtn == LIMITS_LOW_TL){
    if(Ltemp > 0){
      Ltemp--;
    }//end if
    myNextion.setComponentText("t9", String(Ltemp));
//	  EEPROM.write(4, Ltemp);
  }else if(icomBtn == LIMITS_HIGH_TP){
    if((Ltemp <= Htemp)){
      Htemp++;
    }//end if
    myNextion.setComponentText("t10", String(Htemp));
//	  EEPROM.write(6, Htemp);
  }else if(icomBtn == LIMITS_HIGH_TL){
    if((Htemp > 0)&&(Htemp >= Ltemp)){
      Htemp--;
    }//end if
    myNextion.setComponentText("t10", String(Htemp));
//	  EEPROM.write(6, Htemp);
  }else if(icomBtn == LIMITS_HIGH_CO2P){
    if(Lco2 <= Hco2){
      Hco2++;
    }//end if
    myNextion.setComponentText("t14", String(Hco2));
//	  EEPROM.write(8, Hco2);
  }else if(icomBtn == LIMITS_HIGH_CO2L){
    if((Hco2 > 0)&&(Hco2 >= Lco2)){
      Hco2--;
    }//end if
    myNextion.setComponentText("t14", String(Hco2));
//	  EEPROM.write(8, Hco2);
  }else if(icomBtn == LIMITS_LOW_CO2P){
    if(Hco2 > Lco2){
    Lco2++;
    }//end if
    myNextion.setComponentText("t12", String(Lco2));
//	  EEPROM.write(10, Lco2);
  }else if(icomBtn == LIMITS_LOW_CO2L){
    if(Lco2 > 0){
      Lco2--;
    }//end if
    myNextion.setComponentText("t12", String(Lco2));
//	  EEPROM.write(10, Lco2);
  }//end if
  icomBtn = "";
}//end limits

void updateLimits(void){
  myNextion.setComponentText("t7", String(Lhumidity));
  myNextion.setComponentText("t8", String(Hhumidity));
  myNextion.setComponentText("t9", String(Ltemp));
  myNextion.setComponentText("t10", String(Htemp));
  myNextion.setComponentText("t14", String(Hco2));
  myNextion.setComponentText("t14", String(Hco2));
}//end updateLimits
