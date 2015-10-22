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

Mollier::Mollier(uint8_t oneWireBus1, uint8_t oneWireBus2, uint8_t oneWireBus3, uint8_t resolution){
  this->oneWireBus1 = oneWireBus1;//Pin wherein the sensor was placed
  this->oneWireBus2 = oneWireBus2;//Pin wherein the sensor was placed
  this->oneWireBus3 = oneWireBus3;//Pin wherein the sensor was placed
  //  this->oneWireBus4 = oneWireBus4;//Pin wherein the sensor was placed

  this->oneWireTemps1 = new OneWire(this->oneWireBus1);
  this->oneWireTemps2 = new OneWire(this->oneWireBus2);
  this->oneWireTemps3 = new OneWire(this->oneWireBus3);
  //  this->oneWireTemps4 = new OneWire(this->oneWireBus4);

  this->tempSensors1 = new DallasTemperature(oneWireTemps1);
  this->tempSensors2 = new DallasTemperature(oneWireTemps2);
  this->tempSensors3 = new DallasTemperature(oneWireTemps3);
  //  this->tempSensors4 = new DallasTemperature(oneWireTemps4);

  tempSensors1->begin();
  tempSensors2->begin();
  tempSensors3->begin();
  //  tempSensors4->begin();

  tempSensors1->setResolution(resolution);//Global resolution, default 9
  tempSensors2->setResolution(resolution);//Global resolution, default 9
  tempSensors3->setResolution(resolution);//Global resolution, default 9
  //  tempSensors4->setResolution(resolution);//Global resolution, default 9
}//end Mollier

boolean Mollier::requireParasite(void){
  //DallasTemperature library determine and handle if is necesary or no
  if((tempSensors1->isParasitePowerMode())||(tempSensors2->isParasitePowerMode())||(tempSensors3->isParasitePowerMode())){
	return true;
  }else{
	return false;
  }//end if
}//end requireParasite
    
void Mollier::readSensorTemperatures(void){
  tempSensors1->requestTemperatures();// Send the command to get temperatures
  tempSensors2->requestTemperatures();// Send the command to get temperatures
  tempSensors3->requestTemperatures();// Send the command to get temperatures
  uint8_t j = 0;
  uint8_t k = 0;
  for(uint8_t i = 0; i < 10; i++){
	if(i < 4){//Temperatures in degress
	  this->compostTemps[i] = tempSensors1->getTempCByIndex(i);//Compost temperatures
	}else if(i < 7){
	  this->compostTemps[i] = tempSensors2->getTempCByIndex(j);//Compost temperatures
	  j++;
	}else{
	  this->mollierTemps[k] = tempSensors3->getTempCByIndex(k);//First two for Intern, next two for Extern
	  k++;
	}
  }//end for

}//end readSensorTemperatures


void Mollier::getCompostTemperatureSensors(double *compst){//cts means CompostTemperatureSensors
  for(uint8_t i = 0; i < 8; i++){
	compst[i] = compostTemps[i];
  }//end for
}//end getCompostTemperatureSensors
    
void Mollier::getMollierTemperatureSensors(double *compst){//mts means MollierTemperaturesSensors
  for(uint8_t i = 0; i < 2; i++){
	compst[i] = mollierTemps[i];
  }//end for     
}//end getMollierTemperaturesSensors

double Mollier::compostSensorsAverage(void){
  //  double average = 0;
  double orderlyArray[8];
  for(uint8_t i = 0; i< 8; i++){
    orderlyArray[i] = compostTemps[i];
  }//end for
  bubbleSort(orderlyArray);
  //average = (orderlyArray[3]+orderlyArray[4])/2;//
  //  return average;
  return (orderlyArray[3]+orderlyArray[4])/2;//
}//end compostSensorsAverage
    
uint8_t Mollier::mollierCalculus(void){//struct mollierVariables &mollierData
  //uint8_t position;
  //if type = extern then position is 2 otherwise is 0
  //position = (type == "extern") ? 2 : 0;
    
 /* if (mollierTemps[1] > mollierTemps[0]){
	mollierTemps[1] = mollierTemps[0];
  }else if (mollierTemps[1] < 0 || mollierTemps[0] < 0){
	mollierTemps[1] = 0;
	mollierTemps[0] = 0;
  }//end if*/
  
  //Mollier calculus
  mollierData.pva = (6.112 * (exp((17.67 * mollierTemps[1]) / (mollierTemps[1] + 243.5))) - barometricPresure * (mollierTemps[0] - mollierTemps[1]) * 0.00066 * (1 + (0.00115 * mollierTemps[1])));
  mollierData.pvs = (6.112 * exp((17.67 * mollierTemps[0]) / (mollierTemps[0] + 243.5)));
  mollierData.dvt = ((0.00036 * pow(mollierTemps[0], 3)) + 0.00543 * (sq(mollierTemps[0])) + 0.37067 * mollierTemps[0] + 4.81865);
  mollierData.HR = mollierData.pva * 100 / mollierData.pvs;
  mollierData.HA = mollierData.HR * mollierData.dvt / 100;
  mollierData.DEW = (243.5 * log(mollierData.pva / 6.112)) / (17.67 - log(mollierData.pva / 6.112));
  mollierData.DVA = mollierData.HR * mollierData.dvt / 100;
  mollierData.HE = (621.9907 * (mollierData.pva * 100)) / ((barometricPresure * 100) - (mollierData.pva * 100));
      
  return 0;
}//end mollierCalculus

uint8_t Mollier::bubbleSort(double *array){
  for (uint8_t i=1; i<8; i++){
	for(uint8_t j=0 ; j<8 - 1; j++){
	  if (array[j] > array[j+1]){
		double temp = array[j];
		array[j] = array[j+1];
		array[j+1] = temp;                   
	  }//end if
	}//end for
  }//end for
}//end bubblesort
