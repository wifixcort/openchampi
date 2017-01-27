/*
RoomControl
Collection of functions to automate the air conditioning

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

#include "RoomControl.h"

//#define SSerialTxControl 3   //RS485 Direction control
//#define RS485Serial Serial1
//#define RS485Transmit    HIGH
//#define RS485Receive     LOW

boolean stopDamper = false;

//I can use this two variables for fase 1 and 2 because they are mutually exclusive
long incStartT = 0;
long incIntervalT = 1000*60*60*3;//Three hours

/*
//---Motor Speed---
int spdhex1, spdhex2, motorSpeed;
unsigned char uchCRCHi = 0xFF; /* high byte of CRC initialized */
/*unsigned char uchCRCLo = 0xFF; /* low byte of CRC initialized */
/*byte buff[8] = {0x01, 0x06, 0x02, 0xAB, 0x00, 0x00, 0xF9, 0x92};//Default 0Hz


WegVFD *cfw500;

void modBusInit(void){
  cfw500 = new WegVFD(RS485Serial, 19200, SSerialTxControl);
  pinMode(SSerialTxControl, OUTPUT);    
  digitalWrite(SSerialTxControl, RS485Receive);  // Init Transceiver   
}//end modBusInit

void modBusSpeed(float mValue, int limitTemp, int setP, int lowSpeed, int highSpeed, boolean type){
  void modBusInit();
  if(type){
    motorSpeed = cfw500->speedControPLowTemp(mValue, limitTemp, setP, lowSpeed, highSpeed);
  }else{
	motorSpeed = cfw500->speedControPHighTemp(mValue, limitTemp, setP, lowSpeed, highSpeed);

  }//end if
  Serial.print("Temp = ");
  Serial.print(mValue);
  Serial.print("--MSpeed = ");
  Serial.println(motorSpeed);
  cfw500->speedTwoHex(motorSpeed, spdhex1, spdhex2);
  
  buff[4] = spdhex1;
  buff[5] = spdhex2;
 
  //  digitalWrite(pin13Led, HIGH);  // Show activity

  cfw500->CRC16(buff, 6, uchCRCLo, uchCRCHi);//Find CRC+ and CRC-
  buff[6] = uchCRCLo;
  buff[7] = uchCRCHi;  
  cfw500->enableTransmit();

  delay(5);
  RS485Serial.write(buff, 8);
  delay(5);
  
  cfw500->disableTransmit();

  spdhex1 = spdhex2 = 0;
  
  delay(1000);  

}//end modBusSpeed*/

int incubationPhase1(float currentTemp, float lowerLimitTemp, float upperLimitTemp, float outsideTemp, float roomTemp, int maxMotorFrecuency){
  int motorSpeed = 0;
  float tempSetPoint = (lowerLimitTemp+upperLimitTemp)/2;
  unsigned long currentIncT = millis();
  
  if((currentTemp > (tempSetPoint-1))&&(currentTemp < (tempSetPoint+1))){
	stopDamper = false;
	temperatureCloseValves();	
	return 0;
  }else if(currentTemp <= tempSetPoint){//lowerLimitTemp
	if(!stopDamper){
	  if(outsideTemp > roomTemp){
		damperOpen();
	  }else{
		//TODO send request to chiller
		damperClose();
		temperatureIncrease();
		motorSpeed = speedControlPHighTemp(currentTemp, lowerLimitTemp, tempSetPoint, 0, maxMotorFrecuency);
	  }//end if
	}//end if
	if(currentIncT - incStartT > incIntervalT){
	  stopDamper = true;
	  damperClose();
	  //TODO send request to chiller
	  temperatureIncrease();
	  motorSpeed = speedControlPHighTemp(currentTemp, lowerLimitTemp, tempSetPoint, 0, maxMotorFrecuency);
	  incStartT = currentIncT;
	}//end if
  }else if(currentTemp >= tempSetPoint){//upperLimitTemp
	//TODO enfriar
	if(!stopDamper){
	  if(outsideTemp < roomTemp){
		damperOpen();
	  }else{
		//TODO send request to chiller
		damperClose();
		temperatureDecrease(true);
		motorSpeed = speedControlPLowTemp(currentTemp, upperLimitTemp, tempSetPoint, 0, maxMotorFrecuency);
	  }//end if
	}//end if
	if(currentIncT - incStartT > incIntervalT){
	  stopDamper = true;
	  damperClose();
	  //TODO send request to chiller
	  temperatureDecrease(true);
	  motorSpeed = speedControlPLowTemp(currentTemp, upperLimitTemp, tempSetPoint, 0, maxMotorFrecuency);
	  incStartT = currentIncT;
	}//end if
  }else{
	//	Serial.println("NO case");
  }
  return motorSpeed;
}//end incubationPhase1

int incubationPhase2(float currentTemp, float lowerLimitTemp, float upperLimitTemp, float outsideTemp, float roomTemp, float co2Lvl, float lowerCO2Lvl,  float upperCO2Lvl, float externalHumidity, float internalHumidity, float lowerHumidityLvl, float upperHumidityLvl, int maxMotorFrecuency){

  int motorSpeed;
  float tempSetPoint = (lowerLimitTemp+upperLimitTemp)/2;
  //float co2Setpoint = (lowerCO2Lvl+upperCO2Lvl)/2;
  unsigned long currentIncT = millis();
  float humiditySetpoint = (lowerHumidityLvl+upperHumidityLvl)/2;
  if((internalHumidity > (humiditySetpoint-1))&&(internalHumidity < (humiditySetpoint+1))){
	alarmOff();//All is good
	temperatureCloseValves();//Stop heating
  }else if(internalHumidity < humiditySetpoint){
	temperatureCloseValves();//Stop heating
	alarmOn();
	//TODO Display message low Humidity level
  }else if(internalHumidity > humiditySetpoint){
	alarmOn();
	//TODO Display message high Humidity level, correcting!!!
	temperatureIncrease();//tempSetPoint
	motorSpeed = speedControlPHighTemp(currentTemp, lowerLimitTemp, upperLimitTemp, 0, maxMotorFrecuency);
  }//end if

  if(co2Lvl < lowerCO2Lvl){//Close damper CO2 to low
	damperClose();
  }//end if

  if((currentTemp > (tempSetPoint-1))&&(currentTemp < (tempSetPoint+1))){
	stopDamper = false;
	temperatureCloseValves();
	return 0;
  }else if(currentTemp <= tempSetPoint){
	//TODO calentar
	if(!stopDamper){
	  if((outsideTemp > roomTemp)&&(co2Lvl > lowerCO2Lvl)&&(co2Lvl < upperCO2Lvl)){//co2 in ppm
		damperOpen();
	  }else{
		temperatureIncrease();
		motorSpeed = speedControlPHighTemp(currentTemp, lowerLimitTemp, tempSetPoint, 0, maxMotorFrecuency);
	  }//end if
	}//end if
	if(currentIncT - incStartT > incIntervalT){
	  stopDamper = true;
	  damperClose();
	  temperatureIncrease();
	  motorSpeed = speedControlPHighTemp(currentTemp, lowerLimitTemp, tempSetPoint, 0, maxMotorFrecuency);
	  incStartT = currentIncT;
	}//end if	
  }else if(currentTemp >= tempSetPoint){
	//TODO enfriar
	if(!stopDamper){
	  if((outsideTemp < roomTemp)&&(co2Lvl > lowerCO2Lvl)&&(co2Lvl < upperCO2Lvl)){//co2 in ppm
		damperOpen();
	  }else{
		//TODO send request to chiller
		temperatureDecrease(false);
		motorSpeed = speedControlPLowTemp(currentTemp, upperLimitTemp, tempSetPoint, 0, maxMotorFrecuency);
	  }//end if
	}//end if
	if(currentIncT - incStartT > incIntervalT){
	  stopDamper = true;
	  damperClose();
	  temperatureDecrease(false);
	  motorSpeed = speedControlPLowTemp(currentTemp, upperLimitTemp, tempSetPoint, 0, maxMotorFrecuency);
	  incStartT = currentIncT;
	}//end if
  }//end if  
  return motorSpeed;
}//end incubationPhase2

int induction(float currentTemp, float lowerLimitTemp, float upperLimitTemp, float outsideTemp, float roomTemp, float co2Lvl, float lowerCO2Lvl,  float upperCO2Lvl, float externalHumidity, float internalHumidity, float lowerHumidityLvl, float upperHumidityLvl, int maxMotorFrecuency){
  int motorSpeed;
  motorSpeed = incubationPhase2(currentTemp, lowerLimitTemp, upperLimitTemp, outsideTemp, roomTemp, co2Lvl, lowerCO2Lvl, upperCO2Lvl, externalHumidity, internalHumidity, lowerHumidityLvl, upperHumidityLvl, maxMotorFrecuency);
  return motorSpeed;
}//induction

int startGrowth(float currentTemp, float lowerLimitTemp, float upperLimitTemp, float outsideTemp, float roomTemp, float co2Lvl, float lowerCO2Lvl,  float upperCO2Lvl, float externalHumidity, float internalHumidity, float lowerHumidityLvl, float upperHumidityLvl, int maxMotorFrecuency){
  int motorSpeed;
  motorSpeed = incubationPhase2(currentTemp, lowerLimitTemp, upperLimitTemp, outsideTemp, roomTemp, co2Lvl, lowerCO2Lvl, upperCO2Lvl, externalHumidity, internalHumidity, lowerHumidityLvl, upperHumidityLvl, maxMotorFrecuency);
  return motorSpeed;
}//induction


int growth(float currentTemp, float lowerLimitTemp, float upperLimitTemp, float outsideTemp, float roomTemp, float externalHumidity, float internalHumidity, float lowerHumidityLvl, float upperHumidityLvl, int maxMotorFrecuency){

  int motorSpeed;
  float tempSetPoint = (lowerLimitTemp+upperLimitTemp)/2;
  float humiditySetpoint = (lowerHumidityLvl+upperHumidityLvl)/2;
  if((internalHumidity > (humiditySetpoint-1))&&(internalHumidity < (humiditySetpoint+1))){
	alarmOff();//All is good
	temperatureCloseValves();//Stop heating
  }else if(internalHumidity < humiditySetpoint){
	temperatureCloseValves();//Stop heating
	alarmOn();
	//TODO Display message low Humidity level
  }else if(internalHumidity > humiditySetpoint){
	alarmOn();
	//TODO Display message high Humidity level, correcting!!!
	temperatureIncrease();
	motorSpeed = speedControlPHighTemp(currentTemp, lowerLimitTemp, tempSetPoint, 0, maxMotorFrecuency);
	return motorSpeed;//CHECK THIS!!!
  }//end if
  motorSpeed = incubationPhase1(currentTemp, lowerLimitTemp, upperLimitTemp, outsideTemp, roomTemp, maxMotorFrecuency);
  return motorSpeed;
}//end growth

int drying(int maxMotorFrecuency){//Frecuency in Hz
  //TODO set motor speed
  temperatureIncrease();

  return 1;
}//end drying

uint8_t speedControlPHighTemp(float messuredValue, float minTemp, float setPoint, int minSpeed, int maxSpeed){
  int motorSpeed;


  if((messuredValue >= minTemp)&&(messuredValue <= setPoint)){
    motorSpeed = map(messuredValue, minTemp, setPoint, maxSpeed, minSpeed);
  }else if(messuredValue <= setPoint){
    motorSpeed = maxSpeed;
  }else if(messuredValue >= minTemp){
    motorSpeed = 0;  
  }//end if
  
  return motorSpeed;
}//end speedControPHighTemp

uint8_t speedControlPLowTemp(float messuredValue, float maxTemp, float setPoint, int minSpeed, int maxSpeed){
  int motorSpeed;  
  if((messuredValue >= setPoint)&&(messuredValue <= maxTemp)){
    motorSpeed = map(messuredValue, setPoint, maxTemp, minSpeed, maxSpeed);
  }else if(messuredValue <= maxTemp){
    motorSpeed = 0;
  }else if(messuredValue >= setPoint){
    motorSpeed = maxSpeed;  
  }//end if
  
  return motorSpeed;
}//end speedControPLowTemp


