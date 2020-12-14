//------------------------------------------------------------------------------
// @file: temp_sensor.h
// @created on: May 1st, 2020
//
// LICENCE
//------------------------------------------------------------------------------

// IFNDEF ----------------------------------------------------------------------
#ifndef BUZZER_H_
#define BUZZER_H_

// INCLUDES --------------------------------------------------------------------
#include <Arduino.h>

// FOWARD DECLARATIONS ---------------------------------------------------------

// NAMESPACES ------------------------------------------------------------------
namespace xlab {

class Buzzer
{
public:
    // Initialize the buzzer with an attached pin as parameter.
    //
    // @param adcpin[in]: Pin Number
    Buzzer(const uint8_t &buzzer_pin, const uint8_t &pwm_channel);
    ~Buzzer();

    void play(uint32_t frequency, uint32_t duration_ms, uint32_t repetitions);
    void heartBeat();
    bool isPlaying();

private:    
    bool playing_;
    uint8_t buzzerPin_;
    uint8_t pwmChannel_;
    uint32_t startPeriod_;
    int32_t currentReps_;
    uint32_t targetReps_;
    uint32_t targetDuration_;
    uint32_t targetFrequency_;
};

} // namespace xlab

#endif