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

#ifndef _MollierMin_
#define _MollierMin_

#include <Arduino.h>
#include <OneWire.h>
#include <DallasTemperature.h>


class MollierMin{
 private:
  uint8_t oneWireBus3;//2 MollierMine Temperature Sensors
  //  uint8_t oneWireBus4;//1 Multipurpose  temperature sensor
  OneWire *oneWireTemps3;
  //  OneWire *oneWireTemps4;

  DallasTemperature *tempSensors3;
  //  DallasTemperature *tempSensors4;

  double MollierMinTemps[2];
  const float barometricPresure = 1013.25;//float barometricPresure = 1013.25;
  //MollierMinVariables MollierMinData;

 public:
  MollierMin(){}//end MollierMin
  MollierMin(uint8_t oneWireBus3, uint8_t resolution = 9);
  
  struct MollierMinVariables{
    double pva, pvs, dvt, HR, HA, DEW, DVA, HE;
  }MollierMinData;
  
  boolean requireParasite(void);
    
  void readSensorTemperatures(void);
    
  void getMollierMinTemperatureSensors(double *compst);
    
  uint8_t MollierMinCalculus(void);//struct MollierMinVariables &MollierMinData
  
};//end MollierMin

#endif
///----------
