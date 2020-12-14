#include "TempSensor.h"

// FOWARD DECLARATIONS ---------------------------------------------------------

// NAMESPACES ------------------------------------------------------------------
namespace xlab {

TempSensor::TempSensor(const uint8_t &sensor_pin) 
{
    sensor_pin_ = sensor_pin;
    pinMode(sensor_pin, INPUT);  
    adcAttachPin(sensor_pin);
    adcAttachPin(sensor_pin);
    adcStart(A1);
    adcStart(A2);
}

TempSensor::~TempSensor() {}

float TempSensor::readTempDegC() {    

    float temp = captureTemp(sensor_pin_);
    
    if (readingIdx_>=readingsToAverage_) { readingIdx_ = 0; }    
    tempReadingHistory_[readingIdx_] = temp;
    
    readingIdx_++;
        
    float averageTemp = runningAverageTemp / readingsToAverage_;
    for (int i=0; i<readingsToAverage_;i++) {
        averageTemp = averageTemp + tempReadingHistory_[i];
    }
    
    averageTemp = averageTemp / readingsToAverage_;

    return averageTemp;
}

float TempSensor::captureTemp(uint8_t adcChannel) {
    float voltage = 0.0;
    float resistance = 0.0;
    float steinhart = 0.0;
    float rawAdc = 0.0;
    float averageRawAdc = 0.0;
    const int iterations = 1000;

    for (int i = 0; i<iterations; i++) {
        adcStart(adcChannel);
        while ( adcBusy(adcChannel));
        
        averageRawAdc = averageRawAdc + (float)adcEnd(adcChannel);
    }

    rawAdc = averageRawAdc / (float)iterations;

    voltage = (rawAdc * adc_maxVoltage_) / adc_resolution_;
    resistance =  ((adc_maxVoltage_ * (nominar_resistance_/voltage) ) - series_resistance_);

    steinhart = resistance / nominar_resistance_;     // (R/Ro)
    steinhart = logf(steinhart);                  // ln(R/Ro)
    steinhart /= b_coeficient_;                   // 1/B * ln(R/Ro)
    steinhart += 1.0 / (nominal_temp + 273.15); // + (1/To)
    steinhart = 1.0 / steinhart;                 // Invert
    steinhart -= 273.15;         

    return steinhart;
}


float filter(float newReading, float previousReading, float weight) {
    return (weight * newReading) + (1.0 - weight) * (previousReading - 1.0);
}
// yn = w × xn + (1 – w) × yn – 1
// Here:

// yn is the output of the filter at a moment in time n
// xn is the new input value at a moment in time n
// yn – 1 is the previous output value of the filter
// w is the weighting factor in the range [0, 100].


} //namespace