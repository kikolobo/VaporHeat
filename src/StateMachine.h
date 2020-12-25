//------------------------------------------------------------------------------
// @file: temp_sensor.h
// @created on: May 1st, 2020
//
// LICENCE
//------------------------------------------------------------------------------

// IFNDEF ----------------------------------------------------------------------
#ifndef STATE_MACH_H_
#define STATE_MACH_H_

// INCLUDES --------------------------------------------------------------------
#include <Arduino.h>

// FOWARD DECLARATIONS ---------------------------------------------------------

// NAMESPACES ------------------------------------------------------------------
namespace xlab {

class StateMachine
{
public:
    enum class State
        {
            OFF = 0,
            WAITING,
            OVEN_60_HEATING,
            OVEN_90_HEATING,
            OVEN_90_TESTING,
            OVEN_60_TESTING,
            OVEN_60_TEST_READY,
            OVEN_90_TEST_READY,
            OVENS_READY,
            READY_FOR_TESTING
        };

    // Initialize the buzzer with an attached pin as parameter.
    //
    // @param adcpin[in]: Pin Number
    StateMachine();
    ~StateMachine();

    void setState(State state);
    State getState();


private:    
    State currentState_;
};

} // namespace xlab

#endif