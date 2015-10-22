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
#define DEBUG2
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



//---Structures declarations---
struct setPointVariables{
    float sPointValue;
    int sBlv, sBarv, sBox, sI;
    setPointVariables(){sI=0;}//set sI zero default value
};//setPointVariables

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
MQ135 *gasSensor;
//---temperatureChecker---
double *compostSensors;
double *mollierSensors;
//---Weg VFD---
WegVFD *cfw500;
//---Nextion Screen---
Nextion myNextion(nextion, 9600); //create a Nextion object named myNextion using the nextion serial port @ 9600bps

//----Structure definition in Mollier class---
setPointVariables spHumedad;


//float M, relativeHumidity, absouleHumidity, specificHumidity, outdoorHumidity, absoluteHumidityOutside;//variable de HR 

//----------------State Machines Variables-------------------
unsigned long actionRelayStartT, co2PumpStartT, co2SensorStartT, checkerStartT;

unsigned long actionRelayIntervalT = 1000,
              co2PumpIntervalT = 1000,
              checkerIntervalT = 1000;//1s default
//-----------------------------------------------------------

//---Drying the room, manually set---
boolean goToDryTime = true;
unsigned long dryingStartT, dryingTotalT;

//---CO2 Level in the room---
float co2Level = 0;

double averageTemperature = 0;
float externalTemperature = 24;//Standard
float externalHumidity = 4000;//Just a # for debug

//---RoomControl---
uint8_t controlState = 1;
String systemPhase = "incubacion1";
String manualRelay = "";
//=====================================================================

//---Motor Speed Control---
int byteReceived, byteSend, spdhex1, spdhex2, motorSpeed;
unsigned char uchCRCHi = 0xFF; /* high byte of CRC initialized */
unsigned char uchCRCLo = 0xFF; /* low byte of CRC initialized */
byte buff[8] = {0x01, 0x06, 0x02, 0xAB, 0x00, 0x00, 0xF9, 0x92};//Default 0Hz 
int mSpeed = 0;

//---Screen---
uint8_t actualPage = 0;
String actualState = "";
String pastState = "";
boolean coldWater = true, 
        hotWater = true,
        damper = true,
        alarm = true;//Manual relays state
String incommingMsg;

//---Limits--
uint8_t Ltemp, Htemp, Lco2, Hco2, Lhumidity, Hhumidity;

void setup() {

  #if defined(DEBUG)
    Serial.begin(19200);
    Serial.println("Starting OpenChampi Automatization Control...");
    Serial.flush();
  #endif

  #if defined(DEBUG2)
    Serial.begin(19200);
    Serial.println("Starting OpenChampi Automatization Control...");
    Serial.flush();
  #endif 

  //Set all relay pins in a few rows posible as outputs
  for(uint8_t i = 38; i < 46; i++){//8 relays
    pinMode(i,OUTPUT);//Pin Mode
    digitalWrite(i, LOW);//Default state LOW
  }//end for

  for(uint8_t i = 6; i < 14; i++){
    pinMode(i, INPUT_PULLUP);
  }//end for
  myNextion.init();
  compostSensors = new double[8];
  mollierSensors = new double[2];
  calculus = new Mollier(ONE_WIRE_BUS_1, ONE_WIRE_BUS_2, ONE_WIRE_BUS_3);
  gasSensor = new MQ135(CO2SENSOR);
  
  actionRelayIntervalT *= 45;//45 seconds
  co2PumpIntervalT *= 60*30;//Every 30 minutes
  checkerIntervalT *= 60*0.25;//Every 5 minutes

  cfw500 = new WegVFD(RS485Serial, 19200, SSerialTxControl);
  pinMode(SSerialTxControl, OUTPUT);
  digitalWrite(SSerialTxControl, RS485Receive);  // Init Transceiver    
  eepromBack();
}//setup

void loop() {
  serialEvent1();
  serialEvent2();
  if(actualPage == 0){
    updatePage0();
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

  if(incommingMsg == MODE_AUTO){
    controlState = 0;
  }else if(incommingMsg == MODE_MANUAL){
    controlState = 1;
  }else if(incommingMsg == CONTROL_SECADO){
    controlState = 2;
  }

  if(controlState == 0){//Automatic
    updatePage1();
  }else if((actualPage == 2)&&(controlState == 1)){//Manual
    updatePage2();
  }else if(controlState == 2){//Secado
    updateSecado();
  }
  
  modBusSpeed(averageTemperature, motorSpeed);

  
}//end loop

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
    co2Level = gasSensor->getPPM();//
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
  return 1;
}//end temperatureChecker

void serialEvent1(){
  while(RS485Serial.available()) { //Look for data from cfw500
    if (RS485Serial.available()){  //Look for data from other cfw500
      digitalWrite(pin13Led, HIGH);  // Show activity
      byteReceived = RS485Serial.read();// Read received byte
     
      //Serial.print(byteReceived, HEX);// Show on Serial Monitor
     // Serial.print(" ");
      digitalWrite(pin13Led, LOW);  // Show activity   
      byteReceived  = 0;
    }//end if
  }//end while
  //Serial.println();
}//end serialEvent1

void serialEvent2(){
  incommingMsg = myNextion.listen(); //check for message
  if((incommingMsg != "")&&(incommingMsg.length() == 1)){ // if a message is received...
    char pageId[6] = {0x30, 0x31, 0x32, 0x33, 0x34, 0x35};
    for(int i = 0; i< 6; i++){
      if(pageId[i] == incommingMsg[0]){
        actualPage = i;
        EEPROM.write(0, i);//Save page
        Serial.print("msg = ");
        Serial.print(incommingMsg);
        Serial.print(", pagina = ");
        Serial.println(actualPage); 
        break;
      }//end if
    }//end for
  }//end if
  else if(incommingMsg != ""){
    Serial.print("INCOM = ");
    Serial.println(incommingMsg);
    if(actualPage == 1){
      systemPhase = incommingMsg;
    }else if(actualPage == 2){
      manualRelay = incommingMsg;
    }
  }
}

void modBusSpeed(float mValue, uint8_t speed){
//    Serial.print("Temp = ");
//  Serial.print(mValue);
//  Serial.print("--MSpeed = ");
//  Serial.println(speed);
  cfw500->speedTwoHex(speed, spdhex1, spdhex2);
  
  buff[4] = spdhex1;
  buff[5] = spdhex2;
 
  digitalWrite(pin13Led, HIGH);  // Show activity

  cfw500->CRC16(buff, 6, uchCRCLo, uchCRCHi);//Find CRC+ and CRC-
  buff[6] = uchCRCLo;
  buff[7] = uchCRCHi;  
//  Serial.println("Sending message...");  
  cfw500->enableTransmit();

  delay(5);
  RS485Serial.write(buff, 8);
  delay(5);
  
  cfw500->disableTransmit();

    digitalWrite(pin13Led, LOW);  // Show activity 

//while-----...
  #if defined(DEBUG) 
    Serial.println("");
  #endif
  spdhex1 = spdhex2 = 0;

  //cfw500->setFrecuency(speed);
  
  delay(500);//Origin 1000

}//end modBusSpeed

void updatePage0(void){
  myNextion.setComponentText("t10", String(averageTemperature));delay(50);
  myNextion.setComponentText("t11", String(externalTemperature));delay(50);
  myNextion.setComponentText("t12", String(externalHumidity));delay(50);
  myNextion.setComponentText("t13", String(mollierSensors[0]));delay(50);
  myNextion.setComponentText("t14", String(calculus->mollierData.HR));delay(50);
  myNextion.setComponentText("t20", String(calculus->mollierData.pva));delay(50);
  myNextion.setComponentText("t21", String(calculus->mollierData.pvs));delay(50);
  myNextion.setComponentText("t22", String(calculus->mollierData.dvt));delay(50);
  myNextion.setComponentText("t23", String(calculus->mollierData.HR));delay(50);
  myNextion.setComponentText("t24", String(calculus->mollierData.HA));delay(50);
  myNextion.setComponentText("t32", String(calculus->mollierData.DEW));delay(50);
  myNextion.setComponentText("t31", String(calculus->mollierData.DVA));delay(50);
  myNextion.setComponentText("t30", String(calculus->mollierData.HE));delay(50);
  myNextion.setComponentText("t29", String(co2Level));delay(50);
  
}

void updatePage1(void){
  //--------Room Control----------
  //TODO seach type of control
  //controlState = //Take from screen???
  /*In automatic control the user can change limit parameters
  but user can't act directly on the system
  In manual control the user have total control over the system,
  no parameter is controlled automatically */  
      //systemPhase = //Take form screen, from EEPROM or both???   
      if(systemPhase == CONTROL_INCUB_1){
        motorSpeed = incubationPhase1(averageTemperature, 24, 27, mollierSensors[0], externalTemperature);
        //Serial.println("INCUBACION 1");
        pastState = "incubacion 1";
      }else if(systemPhase == CONTROL_INCUB_2){
        motorSpeed =  incubationPhase2(averageTemperature, 25.0, 27.0, externalTemperature, mollierSensors[0], co2Level, 3500, 10000, externalHumidity, calculus->mollierData.HR, 99, 100, 24);
        //Serial.println("INCUBACION 2");
        pastState = "incubacion 2";
      }else if(systemPhase == CONTROL_INDUCTION){
        motorSpeed =  induction(averageTemperature, 19, 20, externalTemperature, mollierSensors[0], co2Level, 1200, 3500, externalHumidity, calculus->mollierData.HR, 88, 91, 24);
        //Serial.println("INDUCCION");
        pastState = "induccion";
      }else if(systemPhase == CONTROL_GROWTH_1){
        mSpeed = startGrowth(averageTemperature, 19, 20, externalTemperature, mollierSensors[0], co2Level, 1200, 1600, externalHumidity, calculus->mollierData.HR, 87, 90, 24);
        //Serial.println("CRECIMIENTO 1");
        pastState = "crecimiento 1";
      }else if(systemPhase == CONTROL_GROWTH_2){
        motorSpeed = growth(averageTemperature, 19, 20, externalTemperature, mollierSensors[0], externalHumidity, calculus->mollierData.HR, 87, 89, 24);    
        //Serial.println("CRECIMIENTO 2");
        pastState = "crecimiento 2";
      }else if(systemPhase == CONTROL_SECADO){
        //Serial.println("SECADO");
        pastState = "secado";
        goToDryTime = true;
        controlState = 2;
      }  
}//end updatePage1

void updatePage2(void){
      if(manualRelay == RELAY_COLD_WATER){//systemPhase == CONTROL_INCUB_1
        if(coldWater){
          Serial.println("Agua On");
          digitalWrite(relays.aguaFriaOn, HIGH);
          digitalWrite(relays.aguaFriaOff, LOW);
          coldWater = false;
        }else{
          Serial.println("Agua off");
          digitalWrite(relays.aguaFriaOn, LOW);
          digitalWrite(relays.aguaFriaOff, HIGH);
          coldWater = true;
        }//end if
      }else if(manualRelay == RELAY_HOT_WATER){
        if(hotWater){
          digitalWrite(relays.aguaCalienteOn, HIGH);
          digitalWrite(relays.aguaCalienteOff, LOW);
          hotWater = false;
        }else{
          digitalWrite(relays.aguaCalienteOn, LOW);
          digitalWrite(relays.aguaCalienteOff, HIGH);
          hotWater = true;
        }//end if
      }else if(manualRelay == RELAY_DAMPER){
        Serial.println("damoter");
        if(damper){
          digitalWrite(relays.damper, HIGH);
          damper = false;
        }else{
          digitalWrite(relays.damper, LOW);
          damper = true;
        }//end if
      }else if(manualRelay == RELAY_ALARM){
        if(alarm){
          digitalWrite(relays.alarm, HIGH);
          alarm = false;
        }else{
          digitalWrite(relays.alarm, LOW);
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
      }
      //si humedad <= 88% cambiar a caso 1
      if(calculus->mollierData.HR <= 0.88){
        controlState = 0;
        unsigned long dryingStopT = millis();
        dryingTotalT = (dryingStartT-dryingStopT)*1000*60;//Time in minutes
        //Serial.println(dryingTotalT);
      }//end if 
}//end updateSecado

void eepromBack(){
  actualPage = EEPROM.read(0);
  String page = "page " + String(actualPage);
  myNextion.sendCommand(page.c_str());

  /*if(Lhumidity != 0)
    Lhumidity = EEPROM.read(1);
  if(Hhumidity != 0)
    Hhumidity = EEPROM.read(2);
  if(Ltemp != 0)
    Ltemp = EEPROM.read(3);
  if(Htemp != 0)
    Htemp = EEPROM.read(4);
  if(Hco2 != 0)
    Hco2 = EEPROM.read(5);
  if(Lco2 != 0)
    Lco2 = EEPROM.read(6);
  */
 
}

void updateLimits(void){
  //uint8_t Ltemp, Htemp, Lco2, Hco2, Lhumidity, Hhumidity;
  if(incommingMsg == LIMITS_LOW_HP){
    Lhumidity++;
//    EEPROM.write(1, Lhumidity);
  }else if(incommingMsg == LIMITS_LOW_HL){
    Lhumidity--;
//    EEPROM.write(1, Lhumidity);
  }else if(incommingMsg == LIMITS_HIGH_HP){
    Hhumidity++;
//    EEPROM.write(2, Hhumidity);
  }else if(incommingMsg == LIMITS_HIGH_HL){
    Hhumidity--;
//    EEPROM.write(2, Hhumidity);
  }else if(incommingMsg == LIMITS_LOW_TP){
    Ltemp++;
//    EEPROM.write(3, Ltemp);
  }else if(incommingMsg == LIMITS_LOW_TL){
    Ltemp--;
//    EEPROM.write(3, Ltemp);
  }else if(incommingMsg == LIMITS_HIGH_TP){
    Htemp++;
//    EEPROM.write(4, Htemp);
  }else if(incommingMsg == LIMITS_HIGH_TL){
    Htemp--;
//    EEPROM.write(4, Htemp);
  }else if(incommingMsg == LIMITS_HIGH_CO2P){
    Hco2++;
//    EEPROM.write(5, Hco2);
  }else if(incommingMsg == LIMITS_HIGH_CO2L){
    Hco2--;
//    EEPROM.write(5, Hco2);
  }else if(incommingMsg == LIMITS_LOW_CO2P){
    Lco2++;
//    EEPROM.write(6, Lco2);
  }else if(incommingMsg == LIMITS_LOW_CO2L){
    Lco2--;
//    EEPROM.write(6, Lco2);
  }
//  myNextion.setComponentText("t10", String(Lhumidity));delay(50);
//  myNextion.setComponentText("t11", String(Hhumidity));delay(50);
//  myNextion.setComponentText("t12", String(Ltemp));delay(50);
//  myNextion.setComponentText("t13", String(Htemp));delay(50);
//  myNextion.setComponentText("t14", String(Lco2));delay(50);
//  myNextion.setComponentText("t20", String(Hco2));delay(50);

}//end updateLimits

