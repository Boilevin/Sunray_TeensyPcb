// Ardumower Sunray 
// Copyright (c) 2013-2020 by Alexander Grau, Grau GmbH
// Licensed GPLv3 for open source use
// or Grau GmbH Commercial License for commercial use (http://grauonline.de/cms2/?page_id=153)

#ifndef MOTOR_H
#define MOTOR_H

#include "pid.h"


// selected motor
enum MotorSelect {MOTOR_LEFT, MOTOR_RIGHT, MOTOR_MOW} ;
typedef enum MotorSelect MotorSelect;


class Motor {
  public:
    float robotPitch;  // robot pitch (rad)
    float wheelBaseCm;  // wheel-to-wheel diameter
    int wheelDiameter;   // wheel diameter (mm)
    int ticksPerRevolution; // ticks per revolution
    float ticksPerCm;  // ticks per cm
    bool activateLinearSpeedRamp;  // activate ramp to accelerate/slow down linear speed?
    bool toggleMowDir; // toggle mowing motor direction each mow motor start?    
    bool motorLeftSwapDir;
    bool motorRightSwapDir;
    bool motorError;
    bool motorLeftOverload; 
    bool motorRightOverload; 
    bool motorMowOverload; 
    bool tractionMotorsEnabled;       
    bool enableMowMotor;
    bool motorMowForwardSet; 
    bool odometryError;    
    unsigned long motorOverloadDuration; // accumulated duration (ms)
    int pwmMax;
    int pwmMaxMow; 
    int pwmMinMow; 
    float  pwmSpeedOffset;
    float mowMotorCurrentAverage;
    float currentFactor;
    bool pwmSpeedCurveDetection;
    long motorLeftTicks;
    long motorRightTicks;
    long motorMowTicks;    
    float linearSpeedSet; // m/s
    float angularSpeedSet; // rad/s
    float motorLeftSense; // left motor current (amps)
    float motorRightSense; // right  motor current (amps)
    //bber
    float motorMowSense;  // mower motor current (amps) 
    float motorMow1Sense;  // mower motor current (amps) 
    float motorMow2Sense;  // mower motor current (amps) 
    float motorMow3Sense;  // mower motor current (amps) 
    float motorMowfaultcurrent; // mower motor error current (amps) 
    float motorMowOverloadCurrent; // mower motor current to generate a driving speed reduction(amps) 
    float motorLeftSenseLP; // left motor current (amps, low-pass)
    float motorRightSenseLP; // right  motor current (amps, low-pass)

    //bber
    float motorMowSenseLP;  // mower motor current = the hihest of the 3 mow motor for overload test (amps, low-pass) 
    float motorMow1SenseLP;  // mower motor current (amps, low-pass) 
    float motorMow2SenseLP;  // mower motor current (amps, low-pass) 
    float motorMow3SenseLP;  // mower motor current (amps, low-pass) 
    PID motorLeftPID;
    PID motorRightPID; 
    float motorsSenseLP; // all motors current (amps, low-pass)
    float motorLeftSenseLPNorm; 
    float motorRightSenseLPNorm;
    unsigned long motorMowSpinUpTime;
    bool motorRecoveryState;    
    void begin();
    void run();      
    void test();
    //bber
    void rollTest();
    void distanceTest();
    void plot();
    void enableTractionMotors(bool enable);
    void setLinearAngularSpeed(float linear, float angular, bool useLinearRamp = true);
    void setMowState(bool switchOn);  
    void setMowMaxPwm( int val ); 
    void stopImmediately(bool includeMowerMotor);
  protected: 
    float motorLeftRpmSet; // set speed
    float motorRightRpmSet;   
    float motorLeftRpmCurr;
    float motorRightRpmCurr;
    float motorMowRpmCurr;    
    float motorLeftRpmCurrLP;
    float motorRightRpmCurrLP;    
    float motorMowRpmCurrLP;    
    float motorLeftRpmLast;
    float motorRightRpmLast;
    
    float motorMowPWMSet;  
    float motorMowPWMCurr; 
    int motorLeftPWMCurr;
    int motorRightPWMCurr;    
    float motorMowPWMCurrLP; 
    float motorLeftPWMCurrLP;
    float motorRightPWMCurrLP;    
    int lastControlTime;    
    unsigned long nextSenseTime;          
    bool recoverMotorFault;
    int recoverMotorFaultCounter;
    unsigned long nextRecoverMotorFaultTime;
    int motorLeftTicksZero;    
    int motorRightTicksZero;    
           
    bool setLinearAngularSpeedTimeoutActive;
    unsigned long setLinearAngularSpeedTimeout;    
    void speedPWM ( int pwmLeft, int pwmRight, int pwmMow );
    void control();    
    bool checkFault();
    void checkOverload();
    bool checkOdometryError();
    bool checkMowRpmFault();
    bool checkCurrentTooHighError();    
    bool checkCurrentTooLowError();
    void sense();
    void dumpOdoTicks(int seconds);    
};


#endif
