//------------------------------------------------------------------------------
// @file: temp_sensor.h
// @created on: May 1st, 2020
//
// LICENCE
//------------------------------------------------------------------------------

// IFNDEF ----------------------------------------------------------------------
#ifndef TEMPERATURE_SENSOR_H_
#define TEMPERATURE_SENSOR_H_

// INCLUDES --------------------------------------------------------------------
#include <Arduino.h>

// FOWARD DECLARATIONS ---------------------------------------------------------

// NAMESPACES ------------------------------------------------------------------
namespace xlab {

class TempSensor
{
public:
    // Initialize the sensor with an ADC Pin
    //
    // @param adcpin[in]: Pin Number
    TempSensor(const uint8_t &sensor_pin);
    ~TempSensor();

    // Read temperature value
    //
    // @return the value un float
    float readTempDegC();

    // Holds an observable temp value
    //
    float temp;

private:
    
    const uint8_t readingsToAverage_ = 100;
    float tempReadingHistory_[100] = { };
    float runningAverageTemp = 0;
    const float nominar_resistance_ = 10000.0;
    const float series_resistance_ = 10000.0;
    const float adc_resolution_ = 4096.0;
    const float adc_maxVoltage_ = 3.3;
    const float b_coeficient_ = 3950.0;
    const float nominal_temp = 25.0;
    uint8_t sensor_pin_ = 0;
    int readingIdx_ = 0;

    float captureTemp(uint8_t adcChannel);
};

} // namespace xlab

#endif