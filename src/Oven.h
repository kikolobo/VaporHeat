//------------------------------------------------------------------------------
// @file: Oven.h
// @created on: Nov 21, 2020
//
// LICENCE
//------------------------------------------------------------------------------

// IFNDEF ----------------------------------------------------------------------
#ifndef OVEN_H_
#define OVEN_H_

// INCLUDES --------------------------------------------------------------------
#include <Arduino.h>
#include "PID_v1.h"
#include "Heater.h"
#include "TempSensor.h"
#include <pidautotuner.h>

namespace xlab
{

    class Oven
    {

    public:
        enum class State
        {
            OFF = 0,
            PREHEAT,
            TRANSITION,
            AWAITING,
            STEADY
        };

        struct PIDConstants
        {
             float Kp = 0.0;
             float Ki = 0.0;
             float Kd = 0.0;
             
             PIDConstants(float p, float i, float d) {
               Kp = p;
               Ki = i;
               Kd = d;
             };
             
        };
        

        Oven(Heater* heater, TempSensor* sensor, PIDConstants pidConstants, uint32_t maxPower = 20, uint32_t farMaxPower = 20, uint32_t stabilizationTimeMS = 30000);
        ~Oven();

        /**
   * Stop Heating | Oven Off
   * @param void
   * */
        void off();

        /**
   * Set target temperature in degrees celcius
   * @param taretTempDegC[degc]: Desired Temperature in deg C
   * */
        void setTemp(float targetTempDegC);

   /**
   * Gets current temperature in DegsC   
   * */
        float getTemp();

     /**
   * Gets the current state
   * */
     State getState();

     /**
   * Gets the current state in string
   * */
     String getStateString();

  /**
   * Should get called as fast as possible!
   * */
        void heartBeat();

  /**
   * used to auto tune the system (Not quite good)
   * */
        void autoTune();

 /**
   * Return the target temperature
   * */
        float getTargetTemp();

     /**
   * Return the target temperature
   * */
        uint32_t getSteadyness();


     /**
   * Return the current power setting
   * */
        uint32_t getPowerOutput();

    /**
   * Adjust PID Constants
   * */
        void tunePID(float kP, float kI, float kD);


     /**
   * Adjust PID Constants
   * */
        void tunePID(PIDConstants pidConstants);

      /**
   * Get PID Constants
   * */
        PIDConstants getPIDConstants();


   /**
   * Is the state Steady?
   * */
     bool isSteady();

   /**
   * Is the state Steady?
   * */
     bool isAwaiting();


     //60
     // double Kp_ = 97;
     // double Ki_ = 1.0;
     // double Kd_ = 0.35; //0.10;

     // //90
     // double Kp_ = 87;
     // double Ki_ = 0.9;
     // double Kd_ = 0.59; //0.10;


     //90
    

    private:
        // FUNCTIONS -----------------------------------------------------------------

          void updatePID();
          double Kp_;
          double Ki_;
          double Kd_;

          
          Heater* heater_;
          TempSensor* tempSensor_;          
          PID* pid_;        
          State state_;

          double pid_Setpoint_, pid_Input_, pid_Output_;
                  

          float targetTemp_ = 0.0;
          float currentTemp_ = 0.0;
          float pidOutputLimit_ = 0.0;
          uint32_t stableCounter_ = 0;
          uint32_t stabilizationTimeMS_ = 0;
          uint32_t beginStabilizationTStamp_ = 0;
          uint32_t farMaxPower_ = 0;


          


    }; // End of class Template

} // namespace xlab

#endif