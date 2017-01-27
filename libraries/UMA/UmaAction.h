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


#ifndef __UMAACTION_H__
#define __UMAACTION_H__

#include <Arduino.h>

struct umaRelay{
  uint8_t coldWaterOn = 38;//Open   Chanel 1
  uint8_t coldWaterOff = 39;//Close Chanel 2
  uint8_t hotWaterOn = 40;//Open Chanel 3
  uint8_t hotWaterOff = 41;//Close Chanel 4
  uint8_t damper = 42; //ON Chanel 5
  //uint8_t pumpCO2 = 43; //ON Chanel 6
  uint8_t alarm = 44; //ON Chanel 7
  //  uint8_t resetVfd = 45; //ON????? Chanel 8
};

uint8_t temperatureIncrease(boolean incubation=false);//temperatureIncrease

uint8_t temperatureDecrease(boolean incubation);//temperatureDecrease

uint8_t temperatureCloseValves(void);//Close heat and cold valves

uint8_t humidityIncrease(float externalHumidity, float internalHumidity);//humidityIncrease

uint8_t humidityDecrease(float externalHumidity, float internalHumidity);//humidityDecrease

uint8_t co2Increase(void);//co2Increase

uint8_t co2Decrease(void);//co2Decrease

uint8_t damperOpen(void);

uint8_t damperClose(void);

uint8_t alarmOn(void);

void alarmOff(void);

#endif /* __UMAACTION_H__ */

