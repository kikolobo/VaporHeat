//------------------------------------------------------------------------------
// @file: StateMachine.cpp
// @created on: Nov 21, 2020
//
// LICENCE
//------------------------------------------------------------------------------
#include "StateMachine.h"
// FOWARD DECLARATIONS ---------------------------------------------------------

// NAMESPACES ------------------------------------------------------------------
namespace xlab
{
    // CLASS IMPLEMENTATION --------------------------------------------------------
    StateMachine::StateMachine() {
        currentState_ = State::OFF;
    }

    StateMachine::~StateMachine(){};

    void StateMachine::setState(State state ) {
        currentState_ = state;
    }

    StateMachine::State StateMachine::getState() {
        return currentState_;
    }
    
} // namespace xlab