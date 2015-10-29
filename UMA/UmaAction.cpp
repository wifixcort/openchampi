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

  Please mantain this license information along with authorship
  and copyright notices in any redistribution of this code
  **********************************************************************************
  */

#include "UmaAction.h"

umaRelay relay;

uint8_t temperatureIncrease(boolean incubation){
  if(incubation){
	//Stop water cooling
	if(digitalRead(relay.coldWaterOn)){
	  digitalWrite(relay.coldWaterOn, LOW);
	}//end if
	if(!digitalRead(relay.coldWaterOff)){//If coldWater relay is HIGH don't do this again
	  digitalWrite(relay.coldWaterOff, HIGH);	
	}//end if
	//TODO decrease vfd speed
  }else{
	//Stop water cooling
	if(digitalRead(relay.coldWaterOn)){
	digitalWrite(relay.coldWaterOn, LOW);
	}//end if
	if(!digitalRead(relay.coldWaterOff)){
	  digitalWrite(relay.coldWaterOff, HIGH);
	}//end if
	//Start heating water
	if(digitalRead(relay.hotWaterOff)){
	  digitalWrite(relay.hotWaterOff, LOW);
	}//end if
	if(!digitalRead(relay.hotWaterOn)){
	  digitalWrite(relay.hotWaterOn, HIGH);	
	}//end if
	//TODO decrease vfd speed
  }//end if
}//temperatureIncrease

uint8_t temperatureDecrease(boolean incubation){
  //Stop heating water
  if(digitalRead(relay.hotWaterOn)){
	digitalWrite(relay.hotWaterOn, LOW);
  }
  if(!digitalRead(relay.hotWaterOff)){
	digitalWrite(relay.hotWaterOff, HIGH);
  }//end if
  //Start water cooling
  if(digitalRead(relay.coldWaterOff)){
    digitalWrite(relay.coldWaterOff, LOW);
  }//end if
  if(!digitalRead(relay.coldWaterOn)){
	digitalWrite(relay.coldWaterOn, HIGH);
  }//end if
}//temperatureDecrease


uint8_t temperatureCloseValves(void){
  //Close heat and cold valves

  if(digitalRead(relay.coldWaterOn)){
	digitalWrite(relay.coldWaterOn, LOW);
  }//end if
  if(digitalRead(relay.hotWaterOn)){
	digitalWrite(relay.hotWaterOn, LOW);
  }//end if  

  if(!digitalRead(relay.coldWaterOff)){
	digitalWrite(relay.coldWaterOff, HIGH);
  }//end if
  if(digitalRead(relay.hotWaterOff)){
	digitalWrite(relay.hotWaterOff, HIGH);
  }//end if
}


uint8_t humidityIncrease(float externalHumidity, float internalHumidity){
  if(externalHumidity > internalHumidity){
	alarmOn();
	if(!digitalRead(relay.damper)){
	  digitalWrite(relay.damper, HIGH);
	}//end if
  }else{
	alarmOn();
  }//end if
}//humidityIncrease

uint8_t humidityDecrease(float externalHumidity, float internalHumidity){
  alarmOff();
  if(externalHumidity > internalHumidity){
	digitalWrite(relay.damper, LOW);//Air recirculation
  }//end if
  temperatureIncrease();
}//humidityDecrease

uint8_t co2Increase(void){
  digitalWrite(relay.damper, LOW);//Air recirculation
}//co2Increase

uint8_t co2Decrease(void){
  if(!digitalRead(relay.damper)){
	digitalWrite(relay.damper, HIGH);
  }//end if
}//co2Decrease

uint8_t damperOpen(void){
  if(!digitalRead(relay.damper)){
	digitalWrite(relay.damper, HIGH);
  }//end if
}//end damperOpen

uint8_t damperClose(void){
  if(digitalRead(relay.damper)){
  digitalWrite(relay.damper, LOW);
  }//end if
}//end damperClose


uint8_t alarmOn(void){
  //TODO send mesage to the screen monitor
  if(!digitalRead(relay.alarm)){
	digitalWrite(relay.alarm, HIGH);
  }//end if 
}//end alarmOn

void alarmOff(void){
  if(digitalRead(relay.alarm)){
	digitalWrite(relay.alarm, LOW);
  }//end if
}//end alarmOff
