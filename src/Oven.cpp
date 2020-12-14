//------------------------------------------------------------------------------
// @file: Oven.h
// @created on: Nov 21, 2020
//
// LICENCE
//------------------------------------------------------------------------------
#include "Oven.h"
// FOWARD DECLARATIONS ---------------------------------------------------------

// NAMESPACES ------------------------------------------------------------------
namespace xlab
{

    #define MAX_STEADY 600
    #define MIN_STEADY 590
    // CLASS IMPLEMENTATION --------------------------------------------------------

    Oven::Oven(Heater* heater, TempSensor* sensor, PIDConstants pidConstants, uint32_t maxPower, uint32_t farMaxPower, uint32_t stabilizationTimeMS)
    { 
        heater_ = heater;
        tempSensor_ = sensor;
        state_ = State::OFF;
        Kp_ = pidConstants.Kp;
        Ki_ = pidConstants.Ki;
        Kd_ = pidConstants.Kd;
        stabilizationTimeMS_ = stabilizationTimeMS;
        pid_ = new PID(&pid_Input_, &pid_Output_, &pid_Setpoint_, Kp_, Ki_, Kd_, DIRECT);
        pid_->SetMode(AUTOMATIC);
        pid_->SetOutputLimits(0, maxPower);
        pid_->SetSampleTime(10);
        pidOutputLimit_ = (float)maxPower;
        farMaxPower_ = farMaxPower;
    }

    Oven::~Oven(){};

    void Oven::heartBeat()
    {

        currentTemp_ = tempSensor_->readTempDegC(); 

        if ((targetTemp_ - currentTemp_ >= 20) && (state_ != State::OFF)) {
            heater_->setPower(farMaxPower_);
            state_ = State::PREHEAT;
            return;
        } else if (state_ == State::PREHEAT) {            
            state_ = State::TRANSITION;
        }

        if (currentTemp_ > 0.0)
        {                      
            pid_Input_ = (double)currentTemp_; 
        }

        pid_->Compute();

        heater_->setPower((unsigned int)pid_Output_); //Set Heater

        if (currentTemp_ >= (targetTemp_ - 0.5) && currentTemp_ <= (targetTemp_ + 0.5))
        { 
            if (stableCounter_<MAX_STEADY) {
                stableCounter_++;      
            }               
        } else {  
                if (stableCounter_>0) {
                    stableCounter_--;      
                }      
        }

        if (stableCounter_>=MIN_STEADY) {
            if (beginStabilizationTStamp_ == 0) { 
                beginStabilizationTStamp_ = millis(); 
                state_=State::AWAITING;
            }

            uint32_t elapsed = millis() - beginStabilizationTStamp_;
            if (elapsed >= stabilizationTimeMS_) {
                state_=State::STEADY;
            }
        }
    }

    uint32_t Oven::getPowerOutput() {
            return heater_->getPower();              
    }

    void Oven::setTemp(float targetTempDegC)
    {
        pid_Setpoint_ = (double)targetTempDegC;
        targetTemp_ = targetTempDegC;
        stableCounter_ = 0;        
        beginStabilizationTStamp_ = 0;
        state_ = State::TRANSITION;
        
        if (targetTemp_ - currentTemp_ >= 20) {
            heater_->setPower(farMaxPower_);
            state_ = State::PREHEAT;
        }

        if (targetTemp_ <= 0.0)
        {
            heater_->setPower(0);
            state_ = State::OFF;
        }
    }

    

    void Oven::autoTune()
    {
        PIDAutotuner tuner = PIDAutotuner();
        tuner.setTargetInputValue(targetTemp_);
        tuner.setLoopInterval(10000);
        tuner.setTuningCycles(30);
        tuner.setOutputRange(0, pidOutputLimit_);
        tuner.setZNMode(PIDAutotuner::ZNModeLessOvershoot);
        tuner.startTuningLoop(micros());

        long microseconds;
        while (!tuner.isFinished())
        {
            Serial.println(tempSensor_->readTempDegC());
            microseconds = micros();

            // Get input value here (temperature, encoder position, velocity, etc)
            float input = (float)tempSensor_->readTempDegC();

            // Call tunePID() with the input value and current time in microseconds
            float output = tuner.tunePID(input, microseconds);

            heater_->setPower((unsigned int)output);

            // This loop must run at the same speed as the PID control loop being tuned
            while (micros() - microseconds < 10000) { delayMicroseconds(1); }            
        }
        heater_->setPower(0);
        float kp = tuner.getKp();
        float ki = tuner.getKi();
        float kd = tuner.getKd();

        Serial.println("------------");
        Serial.println(kp);
        Serial.println(ki);
        Serial.println(kd);
        Serial.println("------------");
        delay(8000);
    }

    

    void Oven::tunePID(float kP, float kI, float kD) 
    {
        pid_->SetTunings((double)kP,(double)kI,(double)kD);
        Kp_ = kP;
        Ki_ = kI;
        Kd_ = kD;
    }

    void Oven::tunePID(PIDConstants pidConstants)
    {        
        Kp_ = pidConstants.Kp;
        Ki_ = pidConstants.Ki;
        Kd_ = pidConstants.Kd;
        
        pid_->SetTunings(Kp_,Ki_,Kd_);       
    }

    float Oven::getTemp()
    {
        return currentTemp_;
    }

    uint32_t Oven::getSteadyness() 
    {
     uint32_t pct = (100 / MIN_STEADY) * stableCounter_;
     if (pct >= 100) { pct = 100; }

     return pct;
    }

    void Oven::off()
    {
        setTemp(0.0);
        state_ = State::OFF;
    }

    float Oven::getTargetTemp() {
        return targetTemp_;
    }   

    Oven::State Oven::getState()
    {
        return state_;
    }

    String Oven::getStateString()
    {

        if (state_==State::OFF) { return "O"; }
        if (state_==State::PREHEAT) { return "P"; }
        if (state_==State::TRANSITION) { return "T"; }
        if (state_==State::AWAITING) { return "A"; }
        if (state_==State::STEADY) { return "S"; }

        return "UNKNOWN";
    }
 
    bool Oven::isSteady()
    {        
        if (state_==State::STEADY) { return true; }
        return false;        
    }

    bool Oven::isAwaiting()
    {        
        if (state_==State::AWAITING) { return true; }
        return false;        
    }

    Oven::PIDConstants Oven::getPIDConstants() {
        PIDConstants pidConstants = PIDConstants(Kp_, Ki_, Kd_);
        return pidConstants;
    }

    
} // namespace xlab