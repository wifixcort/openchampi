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

#ifndef _MOLLIER_
#define _MOLLIER_

#include <Arduino.h>
#include <OneWire.h>
#include <DallasTemperature.h>


class Mollier{
 private:


  uint8_t oneWireBus1;//4 Temperature Sensors
  uint8_t oneWireBus2;//4 Temperature Sensors
  uint8_t oneWireBus3;//2 Molliere Temperature Sensors
  //  uint8_t oneWireBus4;//1 Multipurpose  temperature sensor

  OneWire *oneWireTemps1;
  OneWire *oneWireTemps2;
  OneWire *oneWireTemps3;
  //  OneWire *oneWireTemps4;

  DallasTemperature *tempSensors1;
  DallasTemperature *tempSensors2;
  DallasTemperature *tempSensors3;
  DallasTemperature *tempSensors4;

  float compostTemps[8];//Compost temperatures
  float mollierTemps[2];
  const float barometricPresure = 1013.25;//float barometricPresure = 1013.25;
  //mollierVariables mollierData;
  uint8_t resolution;

  uint8_t bubbleSort(float *array);

 public:
  Mollier(){}//end Mollier
  Mollier(uint8_t oneWireBus1, uint8_t oneWireBus2, uint8_t oneWireBus3, uint8_t resolution = 4);
  
  struct mollierVariables{
    float pva, pvs, dvt, HR, HA, DEW, DVA, HE;
  }mollierData;
  
  boolean requireParasite(void);
    
  void readSensorTemperatures(void);

  void getCompostTemperatureSensors(float *compst);
    
  void getMollierTemperatureSensors(float *compst);
    
  uint8_t mollierCalculus(void);//struct mollierVariables &mollierData
  
  float compostSensorsAverage(void);
};//end Mollier

#endif
///----------
