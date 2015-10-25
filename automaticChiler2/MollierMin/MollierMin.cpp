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

#include "MollierMin.h"

MollierMin::MollierMin(uint8_t oneWireBus3, uint8_t resolution){
  this->oneWireBus3 = oneWireBus3;//Pin wherein the sensor was placed
  //  this->oneWireBus4 = oneWireBus4;//Pin wherein the sensor was placed

  this->oneWireTemps3 = new OneWire(this->oneWireBus3);
  //  this->oneWireTemps4 = new OneWire(this->oneWireBus4);

  this->tempSensors3 = new DallasTemperature(oneWireTemps3);
  //  this->tempSensors4 = new DallasTemperature(oneWireTemps4);

  tempSensors3->begin();
  //  tempSensors4->begin();

  tempSensors3->setResolution(resolution);//Global resolution, default 9
  //  tempSensors4->setResolution(resolution);//Global resolution, default 9
}//end MollierMin

boolean MollierMin::requireParasite(void){
  //DallasTemperature library determine and handle if is necesary or no
  if((tempSensors3->isParasitePowerMode())){
	return true;
  }else{
	return false;
  }//end if
}//end requireParasite
    
void MollierMin::readSensorTemperatures(void){
  tempSensors3->requestTemperatures();// Send the command to get temperatures
  for(uint8_t i = 0; i < 2; i++){
	  this->MollierMinTemps[i] = tempSensors3->getTempCByIndex(i);//First two for Intern, next two for Extern
  }//end for

}//end readSensorTemperatures

    
void MollierMin::getMollierMinTemperatureSensors(double *compst){//mts means MollierMinTemperaturesSensors
  for(uint8_t i = 0; i < 2; i++){
	compst[i] = MollierMinTemps[i];
  }//end for     
}//end getMollierMinTemperaturesSensors
    
uint8_t MollierMin::MollierMinCalculus(void){//struct MollierMinVariables &MollierMinData
  //uint8_t position;
  //if type = extern then position is 2 otherwise is 0
  //position = (type == "extern") ? 2 : 0;
    
 /* if (MollierMinTemps[1] > MollierMinTemps[0]){
	MollierMinTemps[1] = MollierMinTemps[0];
  }else if (MollierMinTemps[1] < 0 || MollierMinTemps[0] < 0){
	MollierMinTemps[1] = 0;
	MollierMinTemps[0] = 0;
  }//end if*/
  
  //MollierMin calculus
  MollierMinData.pva = (6.112 * (exp((17.67 * MollierMinTemps[1]) / (MollierMinTemps[1] + 243.5))) - barometricPresure * (MollierMinTemps[0] - MollierMinTemps[1]) * 0.00066 * (1 + (0.00115 * MollierMinTemps[1])));
  MollierMinData.pvs = (6.112 * exp((17.67 * MollierMinTemps[0]) / (MollierMinTemps[0] + 243.5)));
  MollierMinData.dvt = ((0.00036 * pow(MollierMinTemps[0], 3)) + 0.00543 * (sq(MollierMinTemps[0])) + 0.37067 * MollierMinTemps[0] + 4.81865);
  MollierMinData.HR = MollierMinData.pva * 100 / MollierMinData.pvs;
  MollierMinData.HA = MollierMinData.HR * MollierMinData.dvt / 100;
  MollierMinData.DEW = (243.5 * log(MollierMinData.pva / 6.112)) / (17.67 - log(MollierMinData.pva / 6.112));
  MollierMinData.DVA = MollierMinData.HR * MollierMinData.dvt / 100;
  MollierMinData.HE = (621.9907 * (MollierMinData.pva * 100)) / ((barometricPresure * 100) - (MollierMinData.pva * 100));
      
  return 0;
}//end MollierMinCalculus

