//------------------------------------------------------------------------------
// @file: Oven.h
// @created on: Nov 21, 2020
//
// LICENCE
//------------------------------------------------------------------------------
#include "Buzzer.h"
// FOWARD DECLARATIONS ---------------------------------------------------------

// NAMESPACES ------------------------------------------------------------------
namespace xlab
{
    // CLASS IMPLEMENTATION --------------------------------------------------------
    Buzzer::Buzzer(const uint8_t &buzzer_pin, const uint8_t &pwm_channel) {
        buzzerPin_ = buzzer_pin;
        pwmChannel_ = pwm_channel;
        // pinMode(buzzer_pin, OUTPUT);
        ledcAttachPin(buzzer_pin, pwm_channel);
        ledcSetup(pwm_channel, 4000, 8);
        startPeriod_ = 0;
        currentReps_ = -1;
        playing_ = false;
    }

    Buzzer::~Buzzer(){};

    void Buzzer::play(uint32_t frequency, uint32_t duration_ms, uint32_t repetitions) 
    {
        currentReps_= 0;
        targetReps_ = repetitions;
        targetDuration_ = duration_ms;
        targetFrequency_ = frequency;
        startPeriod_ = millis();        
        ledcWriteTone(pwmChannel_, targetFrequency_); 
        ledcWrite(pwmChannel_, 255);       
        playing_ = true;
    }

    bool Buzzer::isPlaying() 
    {
        return playing_;
    }

    void Buzzer::heartBeat() 
    {

        if ((millis() - startPeriod_) >= targetDuration_ && playing_ == true) {            
            ledcWriteTone(pwmChannel_, 0); //Stop
            ledcWrite(pwmChannel_, 255);
            currentReps_++;
            playing_ = false;
        }
        
        if (currentReps_<=targetReps_ && playing_ == false) {            
            // ledcWriteTone(pwmChannel_, targetFrequency_);
            ledcWrite(pwmChannel_, 255);
            startPeriod_ = millis();
            playing_ = true;            
        }                  
    }
    
} // namespace xlab