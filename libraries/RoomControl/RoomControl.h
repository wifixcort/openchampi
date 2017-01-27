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

#ifndef __ROOMCONTROL_H__
#define __ROOMCONTROL_H__

#include <Arduino.h>
#include "UmaAction.h"
#include "WegVFD.h"

void modBusInit(void);

//void modBusSpeed(float mValue, int limitTemp, int setP, int lowSpeed, int highSpeed, boolean type);

int incubationPhase1(float currentTemp, float lowerLimitTemp, float upperLimitTemp, float outsideTemp, float roomTemp, int maxMotorFrecuency = 40);

int incubationPhase2(float currentTemp, float lowerLimitTemp, float upperLimitTemp, float outsideTemp, float roomTemp, float co2Lvl, float lowerCO2Lvl,  float upperCO2Lvl, float externalHumidity, float internalHumidity, float lowerHumidityLvl, float upperHumidityLvl, int maxMotorFrecuency);

int induction(float currentTemp, float lowerLimitTemp, float upperLimitTemp, float outsideTemp, float roomTemp, float co2Lvl, float lowerCO2Lvl,  float upperCO2Lvl, float externalHumidity, float internalHumidity, float lowerHumidityLvl, float upperHumidityLvl, int maxMotorFrecuency);

int startGrowth(float currentTemp, float lowerLimitTemp, float upperLimitTemp, float outsideTemp, float roomTemp, float co2Lvl, float lowerCO2Lvl,  float upperCO2Lvl, float externalHumidity, float internalHumidity, float lowerHumidityLvl, float upperHumidityLvl, int maxMotorFrecuency);

int growth(float currentTemp, float lowerLimitTemp, float upperLimitTemp, float outsideTemp, float roomTemp, float externalHumidity, float internalHumidity, float lowerHumidityLvl, float upperHumidityLvl, int maxMotorFrecuency);

int drying(int maxMotorFrecuency=24);//Frecuency in Hz

uint8_t speedControlPHighTemp(float messuredValue, float minTemp, float setPoint, int minSpeed, int maxSpeed);

uint8_t speedControlPLowTemp(float messuredValue, float maxTemp, float setPoint, int minSpeed, int maxSpeed);

#endif /* __ROOMCONTROL_H__ */

