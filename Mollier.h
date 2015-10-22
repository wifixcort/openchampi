
///--------
#ifndef _MOLLIER_
#define _MOLLIER_

#include <Arduino.h>
#include <OneWire.h>
#include <DallasTemperature.h>


struct mollierVariables{
  double pva, pvs, dvt, HR, HA, DEW, DVA, HE;
};//mollInterno, mollExterno;


class Mollier{
 private:

  uint8_t oneWireBus;
  OneWire *oneWireTemps;
  DallasTemperature *tempSensors;  
  double compostTemps[6];//Compost temperatures
  double mollierTemps[4];
  const float barometricPresure = 1013.25;//float barometricPresure = 1013.25;
 public:

  Mollier(){}//end Mollier
  Mollier(uint8_t oneWireBus, uint8_t resolution = 9);

  boolean requireParasite(void);
    
  void readSensorTemperatures(void);

  void getCompostTemperatureSensors(double *compst);
    
  void getMollierTemperaturesSensors(double *compst, uint8_t numSensors = 4);
    
  uint8_t mollierCalculus(struct mollierVariables &mollierData, const char *type = "intern");
  
  double compostSensorsAverage(void);
};//end Mollier

#endif
///----------
