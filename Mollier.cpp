

#include "Mollier.h"

Mollier::Mollier(uint8_t oneWireBus, uint8_t resolution){
  this->oneWireBus = oneWireBus;//Pin wherein the sensor was placed
  this->oneWireTemps = new OneWire(this->oneWireBus);
  this->tempSensors = new DallasTemperature(oneWireTemps);
  tempSensors->begin();
  tempSensors->setResolution(resolution);//Global resolution, default 9
}//end Mollier

boolean Mollier::requireParasite(void){
  //DallasTemperature library determine and handle if is necesary or no
  if(tempSensors->isParasitePowerMode()){
	return true;
  }else{
	return false;
  }//end if
}//end requireParasite
    
void Mollier::readSensorTemperatures(void){
  tempSensors->requestTemperatures();// Send the command to get temperatures
  for(uint8_t i = 0; i < 10; i++){
	if(i <6){//Temperatures in degress
	  compostTemps[i] = tempSensors->getTempCByIndex(i);//Compost temperatures
	}else{
	  mollierTemps[i] = tempSensors->getTempCByIndex(i);//First two for Intern, next two for Extern
	}
  }//end for
}//end readSensorTemperatures


void Mollier::getCompostTemperatureSensors(double *compst){//cts means CompostTemperatureSensors
  for(uint8_t i = 0; i < 6; i++){
	compst[i] = compostTemps[i];
  }//end for
}//end getCompostTemperatureSensors
    
void Mollier::getMollierTemperaturesSensors(double *compst, uint8_t numSensors){//mts means MollierTemperaturesSensors
  for(uint8_t i = 0; i < numSensors; i++){
	compst[i] = mollierTemps[i];
  }//end for     
}//end getMollierTemperaturesSensors

double Mollier::compostSensorsAverage(void){
  double average = 0;
  for(uint8_t i = 0; i< 6; i++){
	average += compstTemps[i];
  }//end for
  average /= 6;
  return average;
}//end compostSensorsAverage
    
uint8_t Mollier::mollierCalculus(struct mollierVariables &mollierData, const char *type){
  uint8_t position;
  //if type = extern then position is 2 otherwise is 0
  position = (type == "extern") ? 2 : 0;
      
  if (mollierTemps[position+1] > mollierTemps[position]){
	mollierTemps[position+1] = mollierTemps[position];
  }else if (mollierTemps[position+1] < 0 || mollierTemps[position] < 0){
	mollierTemps[position+1] = 0;
	mollierTemps[position] = 0;
  }//end if

  //Mollier calculus
  mollierData.pva = (6.112 * (exp((17.67 * mollierTemps[position+1]) / (mollierTemps[position+1] + 243.5))) - barometricPresure * (mollierTemps[position] - mollierTemps[position+1]) * 0.00066 * (1 + (0.00115 * mollierTemps[position+1])));
  mollierData.pvs = (6.112 * exp((17.67 * mollierTemps[position]) / (mollierTemps[position] + 243.5)));
  mollierData.dvt = ((0.00036 * pow(mollierTemps[position], 3)) + 0.00543 * (sq(mollierTemps[position])) + 0.37067 * mollierTemps[position] + 4.81865);
  mollierData.HR = mollierData.pva * 100 / mollierData.pvs;
  mollierData.HA = mollierData.HR * mollierData.dvt / 100;
  mollierData.DEW = (243.5 * log(mollierData.pva / 6.112)) / (17.67 - log(mollierData.pva / 6.112));
  mollierData.DVA = mollierData.HR * mollierData.dvt / 100;
  mollierData.HE = (621.9907 * (mollierData.pva * 100)) / ((barometricPresure * 100) - (mollierData.pva * 100));      
      
  return 0;
}//end mollierCalculus
