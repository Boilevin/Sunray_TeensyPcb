// Ardumower Sunray 
// Copyright (c) 2013-2020 by Alexander Grau, Grau GmbH
// Licensed GPLv3 for open source use
// or Grau GmbH Commercial License for commercial use (http://grauonline.de/cms2/?page_id=153)




#ifndef __linux__

  #ifdef _SAM3XA_                 // Arduino Due
       #include "AmRobotDriver.h"
       #include "../due/DueTimer.h"
  #elif __SAMD51__                      // Adafruit Grand Central M4 
       #include "AmRobotDriver.h"
       #include "../agcm4/Adafruit_ZeroTimer.h"    // __SAMD51__
  #elif __IMXRT1062__  //teensy 4
       #include "TeensyRobotDriver.h"
       #include "../mpu/INA226.h"  // for sense on robomow
  #endif

#include "../../config.h"
#include "../../helper.h"
#include "../../robot.h"
#include "../../pinman.h"
#include "../../cpu.h"
#include "../../ArduinoUniqueID.h"

#define SUPER_SPIKE_ELIMINATOR 1  // advanced spike elimination  (experimental, comment out to disable)


volatile int odomTicksLeft  = 0;
volatile int odomTicksRight = 0;
volatile int odomTicksMow = 0;

volatile unsigned long motorLeftTicksTimeout = 0;
volatile unsigned long motorRightTicksTimeout = 0;
volatile unsigned long motorMowTicksTimeout = 0;

volatile unsigned long motorLeftTransitionTime = 0;
volatile unsigned long motorRightTransitionTime = 0;
volatile unsigned long motorMowTransitionTime = 0;

volatile float motorLeftDurationMax = 0;
volatile float motorRightDurationMax = 0;
volatile float motorMowDurationMax = 0;

volatile bool leftPressed = false;
volatile bool rightPressed = false;


volatile boolean tone_pin_state = false;



//bber1
  INA226 ChargeIna226;
  INA226 MotLeftIna226;
  INA226 MotRightIna226;

	INA226 CenterMowIna226;
	INA226 LeftMowIna226;
	INA226 RightMowIna226;

	float shuntvoltagec = 0;
	float shuntvoltagel = 0;
	float shuntvoltager = 0;
	unsigned long nextTimeMotorSense ;
	unsigned long lastTimeMotorMowStuck;
	int motorMowSenseCounter ;
	float Center_Mow_Power;
	float Left_Mow_Power;
	float Right_Mow_Power;
	float Center_Mow_Current;
	float Left_Mow_Current;
	float Right_Mow_Current;
	float Ina226_Bus_Voltage;
  boolean powerboard_I2c_line_Ok;
	//float motorMowPowerOverload=1.0; //Frei 15W, Bis 30W ok, 75W NotAus 30W - 50W auf 1.0 - 0.5 in diese Variable zur Speedreduzierung linear beim MÃ¤hen



void toneHandler(){  
  digitalWrite(pinBuzzer, tone_pin_state= !tone_pin_state);  
}


#if defined(__SAMD51__)
Adafruit_ZeroTimer zerotimer = Adafruit_ZeroTimer(3);

void TC3_Handler() {
  Adafruit_ZeroTimer::timerHandler(3);
}
#endif 




void AmRobotDriver::begin(){
  CONSOLE.println("using robot driver: TeensyRobotDriver");
}



void AmRobotDriver::run(){
}


bool AmRobotDriver::getRobotID(String &id){
  id = "";
  for(size_t i = 0; i < UniqueIDsize; i++){
      byte v = UniqueID[i];
      if (v <= 0xF) id += F("0");
      id += String(v, HEX);  
      if (i + 1 < UniqueIDsize) id += ":";
  }
  return true;
}

bool AmRobotDriver::getMcuFirmwareVersion(String &name, String &ver){
  name = "XX";
  ver = "XX";
  return true;
}

float AmRobotDriver::getCpuTemperature(){
  return GetCPUTemperature();  
}


// ------------------------------------------------------------------------------------


// odometry signal change interrupt

void OdometryMowISR(){	
  return; //no odometry on mow	motor for teensy	  
 // if (digitalRead(pinMotorMowRpm) == LOW) return;
  if (millis() < motorMowTicksTimeout) return; // eliminate spikes  
  #ifdef SUPER_SPIKE_ELIMINATOR
    unsigned long duration = millis() - motorMowTransitionTime;
    if (duration > 5) duration = 0;
    motorMowTransitionTime = millis();
    motorMowDurationMax = 0.7 * max(motorMowDurationMax, ((float)duration));
    motorMowTicksTimeout = millis() + motorMowDurationMax;
  #else
    motorMowTicksTimeout = millis() + 1;
  #endif
  odomTicksMow++;    
}


void OdometryLeftISR(){			  
  if (digitalRead(pinOdometryLeft) == LOW) return;
  if (millis() < motorLeftTicksTimeout) return; // eliminate spikes  
  #ifdef SUPER_SPIKE_ELIMINATOR
    unsigned long duration = millis() - motorLeftTransitionTime;
    if (duration > 5) duration = 0;
    motorLeftTransitionTime = millis();
    motorLeftDurationMax = 0.7 * max(motorLeftDurationMax, ((float)duration));
    motorLeftTicksTimeout = millis() + motorLeftDurationMax;
  #else
    motorLeftTicksTimeout = millis() + 1;
  #endif
  odomTicksLeft++;    
}

void OdometryRightISR(){			
  if (digitalRead(pinOdometryRight) == LOW) return;  
  if (millis() < motorRightTicksTimeout) return; // eliminate spikes
  #ifdef SUPER_SPIKE_ELIMINATOR
    unsigned long duration = millis() - motorRightTransitionTime;
    if (duration > 5) duration = 0;  
    motorRightTransitionTime = millis();
    motorRightDurationMax = 0.7 * max(motorRightDurationMax, ((float)duration));  
    motorRightTicksTimeout = millis() + motorRightDurationMax;
  #else
    motorRightTicksTimeout = millis() + 1;
  #endif
  odomTicksRight++;        
  
  #ifdef TEST_PIN_ODOMETRY
    testValue = !testValue;
    digitalWrite(pinKeyArea2, testValue);  
  #endif
}


AmMotorDriver::AmMotorDriver(){

  // default values for all motor drivers (for parameters description, see AmRobotDriver.h)
  
  // MC33926 (https://www.nxp.com/docs/en/data-sheet/MC33926.pdf) - PwmFreqMax=20 khz
  MC33926.driverName = "MC33926";
  MC33926.forwardPwmInvert = false;
  MC33926.forwardDirLevel = LOW;
  MC33926.reversePwmInvert = true;
  MC33926.reverseDirLevel = HIGH;
  MC33926.usePwmRamp = false;
  MC33926.faultActive = LOW;
  MC33926.resetFaultByToggleEnable = true;
  MC33926.enableActive = HIGH;
  MC33926.disableAtPwmZeroSpeed=false;  
  MC33926.keepPwmZeroSpeed = true;
  MC33926.minPwmSpeed = 0;
  MC33926.maxPwmSpeed = 255;
  MC33926.pwmFreq = PWM_FREQ_3900;
  MC33926.adcVoltToAmpOfs = 0;
  MC33926.adcVoltToAmpScale = 1.905 * 2; // ADC voltage to amp for 2 drivers connected in parallel
  MC33926.adcVoltToAmpPow = 1.0;

// BTS7960 IBT2 DC Motor driver PwmFreqMax=20 khz
  BTS7960.driverName = "MC33926";
  BTS7960.forwardPwmInvert = false;
  BTS7960.forwardDirLevel = LOW;
  BTS7960.reversePwmInvert = true;
  BTS7960.reverseDirLevel = HIGH;
  BTS7960.usePwmRamp = false;
  BTS7960.faultActive = LOW;
  BTS7960.resetFaultByToggleEnable = true;
  BTS7960.enableActive = HIGH;
  BTS7960.disableAtPwmZeroSpeed=false;  
  BTS7960.keepPwmZeroSpeed = true;
  BTS7960.minPwmSpeed = 0;
  BTS7960.maxPwmSpeed = 255;
  BTS7960.pwmFreq = 10000;
  BTS7960.adcVoltToAmpOfs = 0;
  BTS7960.adcVoltToAmpScale = 1.905 * 2; // ADC voltage to amp for 2 drivers connected in parallel
  BTS7960.adcVoltToAmpPow = 1.0;


  // DRV8308 (https://www.ti.com/lit/ds/symlink/drv8308.pdf) - PwmFreqMax=30 khz
  DRV8308.driverName = "DRV8308";
  DRV8308.forwardPwmInvert = false;
  DRV8308.forwardDirLevel = LOW;
  DRV8308.reversePwmInvert = false;
  DRV8308.reverseDirLevel = HIGH;
  DRV8308.usePwmRamp = false;
  DRV8308.faultActive = LOW;
  DRV8308.resetFaultByToggleEnable = true;
  DRV8308.enableActive = LOW;
  DRV8308.disableAtPwmZeroSpeed=false;
  DRV8308.keepPwmZeroSpeed = false; // never go to zero PWM (driver requires a periodic signal)  
  DRV8308.minPwmSpeed = 2;
  DRV8308.maxPwmSpeed = 255;
  DRV8308.pwmFreq = PWM_FREQ_29300;
  DRV8308.adcVoltToAmpOfs = -1.65;   // brushless-adapter: 0A=1.65V, resolution 132mV/A 
  DRV8308.adcVoltToAmpScale = 7.57; 
  DRV8308.adcVoltToAmpPow = 1.0; 

  // A4931 (https://www.allegromicro.com/en/Products/Motor-Driver-And-Interface-ICs/Brushless-DC-Motor-Drivers/~/media/Files/Datasheets/A4931-Datasheet.ashx) - PwmFreqMax=30 kHz
  // alternatives: MS4931 (https://datasheet.lcsc.com/lcsc/1809131539_Hangzhou-Ruimeng-Tech-MS4931_C231944.pdf)
  A4931.driverName = "A4931";
  A4931.forwardPwmInvert = false;
  A4931.forwardDirLevel = HIGH;
  A4931.reversePwmInvert = false;
  A4931.reverseDirLevel = LOW;
  A4931.usePwmRamp = false;
  A4931.faultActive = LOW;
  A4931.resetFaultByToggleEnable = false;
  A4931.enableActive = LOW;  // 'enable' actually is driver brake
  A4931.disableAtPwmZeroSpeed=false;
  A4931.keepPwmZeroSpeed = true;  
  A4931.minPwmSpeed = 0;    
  A4931.maxPwmSpeed = 255;    
  A4931.pwmFreq = PWM_FREQ_29300;   
  A4931.adcVoltToAmpOfs = -1.65;    // brushless-adapter: 0A=1.65V, resolution 132mV/A
  A4931.adcVoltToAmpScale = 7.57;
  A4931.adcVoltToAmpPow = 1.0; 

  // ACT-8015A brushless driver 
  BLDC8015A.driverName = "BLDC8015A";    // just a name for your driver
  BLDC8015A.forwardPwmInvert = false; // invert PWM signal for forward? (false or true)
  BLDC8015A.forwardDirLevel = LOW;    // logic level for forward (LOW or HIGH)
  BLDC8015A.reversePwmInvert = false; // invert PWM signal for reverse? (false or true)
  BLDC8015A.reverseDirLevel = HIGH;   // logic level for reverse (LOW or HIGH)
  BLDC8015A.usePwmRamp = false;       // use a ramp to get to PWM value?    
  BLDC8015A.faultActive = LOW;        // fault active level (LOW or HIGH)
  BLDC8015A.resetFaultByToggleEnable = false; // reset a fault by toggling enable? 
  BLDC8015A.enableActive = HIGH;       // enable active level (LOW or HIGH)
  BLDC8015A.disableAtPwmZeroSpeed = false;  // disable driver at PWM zero speed? (brake function)
  BLDC8015A.keepPwmZeroSpeed = true;  // keep PWM zero value (disregard minPwmSpeed at zero speed)?
  BLDC8015A.minPwmSpeed = 0;          // minimum PWM speed your driver can operate
  BLDC8015A.maxPwmSpeed = 255;            
  BLDC8015A.pwmFreq = PWM_FREQ_29300;  // choose between PWM_FREQ_3900 and PWM_FREQ_29300 here   
  BLDC8015A.adcVoltToAmpOfs = -1.65;      // ADC voltage to amps (offset)    // brushless-adapter: 0A=1.65V, resolution 132mV/A  
  BLDC8015A.adcVoltToAmpScale = 7.57; // ADC voltage to amps (scale)
  BLDC8015A.adcVoltToAmpPow = 1.0;    // ADC voltage to amps (power of number)

  // JYQD brushless driver 
  JYQD.driverName = "JYQD";    // just a name for your driver
  JYQD.forwardPwmInvert = false; // invert PWM signal for forward? (false or true)
  JYQD.forwardDirLevel = LOW;    // logic level for forward (LOW or HIGH)
  JYQD.reversePwmInvert = false; // invert PWM signal for reverse? (false or true)
  JYQD.reverseDirLevel = HIGH;   // logic level for reverse (LOW or HIGH)
  JYQD.usePwmRamp = false;       // use a ramp to get to PWM value?    
  JYQD.faultActive = LOW;        // fault active level (LOW or HIGH)
  JYQD.resetFaultByToggleEnable = true; // reset a fault by toggling enable? 
  JYQD.enableActive = HIGH;       // enable active level (LOW or HIGH)
  JYQD.disableAtPwmZeroSpeed = false;  // disable driver at PWM zero speed? (brake function)
  JYQD.keepPwmZeroSpeed = false;  // keep PWM zero value (disregard minPwmSpeed at zero speed)?
  JYQD.minPwmSpeed = 0;          // minimum PWM speed your driver can operate
  JYQD.maxPwmSpeed = 255;            
  JYQD.pwmFreq = PWM_FREQ_3900;  // choose between PWM_FREQ_3900 and PWM_FREQ_29300 here   
  JYQD.adcVoltToAmpOfs = -1.65;      // ADC voltage to amps (offset)   // brushless-adapter: 0A=1.65V, resolution 132mV/A
  JYQD.adcVoltToAmpScale = 7.57; // ADC voltage to amps (scale)
  JYQD.adcVoltToAmpPow = 1.0;    // ADC voltage to amps (power of number)

  // your custom brushed/brushless driver (ACT-8015A, JYQD_V7.3E3, etc.)
  CUSTOM.driverName = "CUSTOM";    // just a name for your driver
  CUSTOM.forwardPwmInvert = false; // invert PWM signal for forward? (false or true)
  CUSTOM.forwardDirLevel = LOW;    // logic level for forward (LOW or HIGH)
  CUSTOM.reversePwmInvert = false; // invert PWM signal for reverse? (false or true)
  CUSTOM.reverseDirLevel = HIGH;   // logic level for reverse (LOW or HIGH)
  CUSTOM.usePwmRamp = false;       // use a ramp to get to PWM value?    
  CUSTOM.faultActive = LOW;        // fault active level (LOW or HIGH)
  CUSTOM.resetFaultByToggleEnable = false; // reset a fault by toggling enable? 
  CUSTOM.enableActive = LOW;       // enable active level (LOW or HIGH)
  CUSTOM.disableAtPwmZeroSpeed=false;  // disable driver at PWM zero speed? (brake function)
  CUSTOM.keepPwmZeroSpeed = false;  // keep PWM zero value (disregard minPwmSpeed at zero speed)?
  CUSTOM.minPwmSpeed = 0;          // minimum PWM speed your driver can operate
  CUSTOM.maxPwmSpeed = 255;          
  CUSTOM.pwmFreq = PWM_FREQ_29300;  // choose between PWM_FREQ_3900 and PWM_FREQ_29300 here   
  CUSTOM.adcVoltToAmpOfs = -1.65;      // ADC voltage to amps (offset)        // brushless-adapter: 0A=1.65V, resolution 132mV/A
  CUSTOM.adcVoltToAmpScale = 7.57; // ADC voltage to amps (scale)
  CUSTOM.adcVoltToAmpPow = 1.0;    // ADC voltage to amps (power of number)
}
    

void AmMotorDriver::begin(){      

  #ifdef MOTOR_DRIVER_BRUSHLESS    
    CONSOLE.println("MOTOR_DRIVER_BRUSHLESS");    

    // All motors (gears, mow) are assigned individual motor drivers here.
    // NOTE: you can adjust/override default motor driver parameters here if required for a certain motor!
    // example: mowDriverChip.minPwmSpeed = 40; 

    #ifdef MOTOR_DRIVER_BRUSHLESS_MOW_DRV8308  
      mowDriverChip = DRV8308;
    #elif MOTOR_DRIVER_BRUSHLESS_MOW_A4931 
      mowDriverChip = A4931;
      mowDriverChip.minPwmSpeed = 40;
      mowDriverChip.keepPwmZeroSpeed = true;
      mowDriverChip.disableAtPwmZeroSpeed = true;  
      mowDriverChip.usePwmRamp = false;
    #elif MOTOR_DRIVER_BRUSHLESS_MOW_BLDC8015A 
      mowDriverChip = BLDC8015A;    
    #elif MOTOR_DRIVER_BRUSHLESS_MOW_JYQD
      mowDriverChip = JYQD;
    #else 
      mowDriverChip = CUSTOM;
    #endif
    
    #ifdef MOTOR_DRIVER_BRUSHLESS_GEARS_DRV8308  
      gearsDriverChip = DRV8308;                         
    #elif MOTOR_DRIVER_BRUSHLESS_GEARS_A4931 
      gearsDriverChip = A4931;
    #elif MOTOR_DRIVER_BRUSHLESS_GEARS_BLDC8015A
      gearsDriverChip = BLDC8015A;    
    #elif MOTOR_DRIVER_BRUSHLESS_GEARS_JYQD
      gearsDriverChip = JYQD;
    #else 
      gearsDriverChip = CUSTOM;
    #endif

  #else //DC MOTOR DRIVER
    CONSOLE.println("MOTOR_DRIVER_DC");  
    #ifdef MOTOR_DRIVER_MC33926 
      CONSOLE.println("MOTOR_DRIVER_MC33926");    
      mowDriverChip = MC33926;
      gearsDriverChip = MC33926;
    #elif MOTOR_DRIVER_BTS7960
      CONSOLE.println("MOTOR_DRIVER_BTS7960");    
      mowDriverChip = BTS7960;
      gearsDriverChip = BTS7960;
    #endif

  #endif


  // left wheel motor
  pinMode(pinMotorEnable, OUTPUT);
  digitalWrite(pinMotorEnable, gearsDriverChip.enableActive);
  pinMode(pinMotorLeftPWM, OUTPUT);
  pinMode(pinMotorLeftDir, OUTPUT);

  analogWriteFrequency(pinMotorLeftPWM, gearsDriverChip.pwmFreq);
  analogWriteFrequency(pinMotorLeftDir, gearsDriverChip.pwmFreq);
 // pinMode(pinMotorLeftSense, INPUT);
 // pinMode(pinMotorLeftFault, INPUT);

  // right wheel motor
  pinMode(pinMotorRightPWM, OUTPUT);
  pinMode(pinMotorRightDir, OUTPUT);
  
  analogWriteFrequency(pinMotorRightPWM, gearsDriverChip.pwmFreq);
  analogWriteFrequency(pinMotorRightDir, gearsDriverChip.pwmFreq);



 // pinMode(pinMotorRightSense, INPUT);
 // pinMode(pinMotorRightFault, INPUT);

  // mower motor
  pinMode(pinMotorMowDir, OUTPUT);
  pinMode(pinMotorMowPWM, OUTPUT);
  //bber1
  analogWriteFrequency(pinMotorMowPWM, mowDriverChip.pwmFreq);
  analogWriteFrequency(pinMotorMowDir, mowDriverChip.pwmFreq);
 // pinMode(pinMotorMowSense, INPUT);
 // pinMode(pinMotorMowRpm, INPUT);
 // pinMode(pinMotorMowRpm, INPUT_PULLUP);  
  pinMode(pinMotorMowEnable, OUTPUT);
  digitalWrite(pinMotorMowEnable, mowDriverChip.enableActive);
 // pinMode(pinMotorMowFault, INPUT);

  // odometry
  pinMode(pinOdometryLeft, INPUT_PULLUP);
  //pinMode(pinOdometryLeft2, INPUT_PULLUP);
  pinMode(pinOdometryRight, INPUT_PULLUP);
  //pinMode(pinOdometryRight2, INPUT_PULLUP);

  // lift sensor
 // pinMode(pinLift, INPUT_PULLUP);

  // enable interrupts
  attachInterrupt(pinOdometryLeft, OdometryLeftISR, CHANGE);  
  attachInterrupt(pinOdometryRight, OdometryRightISR, CHANGE);  
 // attachInterrupt(pinMotorMowRpm, OdometryMowISR, CHANGE);  
    
	//pinMan.setDebounce(pinOdometryLeft, 100);  // reject spikes shorter than usecs on pin
	//pinMan.setDebounce(pinOdometryRight, 100);  // reject spikes shorter than usecs on pin	
  
  leftSpeedSign = rightSpeedSign = mowSpeedSign = 1;
  lastRightPwm = lastLeftPwm = lastMowPwm = 0;

  CONSOLE.print ("Starting Ina226 current sensor ");
  //remember 2 i2c line with 3 ina226 on them ,soit same i2c adress 2 time
  MotLeftIna226.begin(0x41);
  ChargeIna226.begin(0x40);
  MotRightIna226.begin(0x44);
  CenterMowIna226.begin_I2C1(0x40);  //MOW1 is connect on I2C1
  LeftMowIna226.begin_I2C1(0x41);  //MOW2 is connect on I2C1
  RightMowIna226.begin_I2C1(0x44);  //MOW3 is connect on I2C1

  CONSOLE.print ("Checking  ina226 current sensor connection");
  //check sense powerboard i2c connection
  powerboard_I2c_line_Ok = true;
  if (!ChargeIna226.isConnected(0x40)) {
    CONSOLE.print("INA226 Battery Charge is not OK");
    powerboard_I2c_line_Ok = false;
  }
  if (!MotRightIna226.isConnected(0x44)) {
    CONSOLE.print("INA226 Motor Right is not OK");
    powerboard_I2c_line_Ok = false;
  }
  if (!MotLeftIna226.isConnected(0x41)) {
    CONSOLE.print("INA226 Motor Left is not OK");
    powerboard_I2c_line_Ok = false;
  }
  if (!CenterMowIna226.isConnected_I2C1(0x40)) {
    CONSOLE.print("INA226 MOW1 is not OK");
    powerboard_I2c_line_Ok = false;
  }
  if ( (!LeftMowIna226.isConnected_I2C1(0x41))) {
    CONSOLE.print("INA226 MOW2 is not OK");
    powerboard_I2c_line_Ok = false;
  }
  if ((!RightMowIna226.isConnected_I2C1(0x44))) {
    CONSOLE.print("INA226 MOW3 is not OK");
    powerboard_I2c_line_Ok = false;
  }


  if (powerboard_I2c_line_Ok)
  {
    CONSOLE.print ("Ina226 Begin OK ");
    // Configure INA226


    ChargeIna226.configure(INA226_AVERAGES_4, INA226_BUS_CONV_TIME_1100US, INA226_SHUNT_CONV_TIME_1100US, INA226_MODE_SHUNT_BUS_CONT);
    MotLeftIna226.configure(INA226_AVERAGES_4, INA226_BUS_CONV_TIME_1100US, INA226_SHUNT_CONV_TIME_1100US, INA226_MODE_SHUNT_BUS_CONT);
    MotRightIna226.configure(INA226_AVERAGES_4, INA226_BUS_CONV_TIME_1100US, INA226_SHUNT_CONV_TIME_1100US, INA226_MODE_SHUNT_BUS_CONT);
    //I2C1 bus
    CenterMowIna226.configure_I2C1(INA226_AVERAGES_4, INA226_BUS_CONV_TIME_1100US, INA226_SHUNT_CONV_TIME_1100US, INA226_MODE_SHUNT_BUS_CONT);
    LeftMowIna226.configure_I2C1(INA226_AVERAGES_4, INA226_BUS_CONV_TIME_1100US, INA226_SHUNT_CONV_TIME_1100US, INA226_MODE_SHUNT_BUS_CONT);
    RightMowIna226.configure_I2C1(INA226_AVERAGES_4, INA226_BUS_CONV_TIME_1100US, INA226_SHUNT_CONV_TIME_1100US, INA226_MODE_SHUNT_BUS_CONT);

    CONSOLE.print ("Ina226 Configure OK ");
    // Calibrate INA226. Rshunt = 0.01 ohm, Max excepted current = 4A
    ChargeIna226.calibrate(0.02, 4);
    MotLeftIna226.calibrate(0.02, 4);
    MotRightIna226.calibrate(0.02, 4);
    //I2C1 bus
    CenterMowIna226.calibrate_I2C1(0.02, 4);
    LeftMowIna226.calibrate_I2C1(0.02, 4);
    RightMowIna226.calibrate_I2C1(0.02, 4);

    CONSOLE.print ("Ina226 Calibration OK ");
  }
  else
  {
    CONSOLE.print ("************** WARNING **************");
    CONSOLE.print ("INA226 powerboard connection is not OK");
  }


}


void AmMotorDriver::run(){
}


// brushed/brushless motor driver
//(8-bit PWM=255, 10-bit PWM=1023)
// example logic:
//   IN1 PinPWM         IN2 PinDir
//   PWM                L     Forward
//   PWM                H     Reverse

//warning for BTS7960 motor driver
// Dir pin is used for reverse PWM signal
// Pwm pin is used for forward PWM signal
// 2 EN_R an L are shunt to only one enable possibility ,so never send PWM on both pinDIR and PinPWM at same time
//VCC --> 3.3V
//GND --> GND
//R_IS --> NC
//L_IS --> NC
//R_EN --> pinMotorRightEnable
//L_EN --> pinMotorRightEnable
//RPWM --> pinMotorRightPWM
//LPWM --> pinMotorRightDir
//
// so the change is :
//the dir pin is not a boolean it is use as PWM to reverse the motor = rotate CCW
//the pwm pin is used to go forward = rotate CW

void AmMotorDriver::setMotorDriver(int pinDir, int pinPWM, int speed, DriverChip &chip, int speedSign) {
  //DEBUGLN(speed);
  bool reverse = (speedSign < 0); 
  if ((speed == 0) && (chip.keepPwmZeroSpeed)) {
    #ifdef MOTOR_DRIVER_BTS7960 
    pinMan.analogWrite(pinPWM, 0, chip.pwmFreq);
    pinMan.analogWrite(pinDir, 0, chip.pwmFreq);
    #endif
    // driver does not require periodic signal at zero speed, we can output 'silence' for zero speed    
  } else {
    // verhindert dass das PWM Signal 0 wird. Der Driver braucht einen kurzen Impuls um das PWM zu erkennen.
    // Wenn der z.B. vom max. PWM Wert auf 0 bzw. das Signal auf Low geht, behält er den vorherigen Wert bei und der Motor stoppt nicht
    if (abs(speed) < chip.minPwmSpeed) speed = chip.minPwmSpeed * speedSign;
    if (abs(speed) > chip.maxPwmSpeed) speed = chip.maxPwmSpeed * speedSign;  
  }

  #ifdef MOTOR_DRIVER_BTS7960 
  if (reverse) {  
    // reverse
    pinMan.analogWrite(pinPWM, ((byte)abs(speed)), chip.pwmFreq);  // nPWM (inverted pwm)
    pinMan.analogWrite(pinDir, 0, chip.pwmFreq);       // PWM
  } else {
    // forward
    pinMan.analogWrite(pinPWM, 0, chip.pwmFreq);  // nPWM (inverted pwm)
    pinMan.analogWrite(pinDir, ((byte)abs(speed)), chip.pwmFreq);       // PWM
  }
  #else  // other motor driver
  if (reverse) {  
    //CONSOLE.print("reverse ");
    //CONSOLE.print(pinDir);
    //CONSOLE.print(",");
    //CONSOLE.print(pinPWM);
    //CONSOLE.print(",");
    //CONSOLE.println(speed);    
    // reverse
    digitalWrite(pinDir, chip.reverseDirLevel) ;
    if (chip.reversePwmInvert) 
      pinMan.analogWrite(pinPWM, 255 - ((byte)abs(speed)), chip.pwmFreq);  // nPWM (inverted pwm)
    else 
      pinMan.analogWrite(pinPWM, ((byte)abs(speed)), chip.pwmFreq);       // PWM

  } else {
    //CONSOLE.print("forward ");
    //CONSOLE.print(pinDir);
    //CONSOLE.print(",");
    //CONSOLE.print(pinPWM);
    //CONSOLE.print(",");
    //CONSOLE.println(speed);    
    // forward
    digitalWrite(pinDir, chip.forwardDirLevel) ;
    if (chip.forwardPwmInvert) 
      pinMan.analogWrite(pinPWM, 255 - ((byte)abs(speed)), chip.pwmFreq);  // nPWM (inverted pwm)
    else 
      pinMan.analogWrite(pinPWM, ((byte)abs(speed)), chip.pwmFreq);       // PWM
  }
  #endif  
}
    
void AmMotorDriver::setMotorPwm(int leftPwm, int rightPwm, int mowPwm){  
  // remember speed sign during zero-transition
  if (leftPwm < 0) leftSpeedSign = -1;
  if (leftPwm > 0) leftSpeedSign = 1;
  if (rightPwm < 0) rightSpeedSign = -1;
  if (rightPwm > 0) rightSpeedSign = 1;
  if (mowPwm < 0) mowSpeedSign = -1;
  if (mowPwm > 0) mowSpeedSign = 1;   
  
  // limit pwm to ramp if required
  if (gearsDriverChip.usePwmRamp){
    int deltaLeftPwm = leftPwm-lastLeftPwm;
    leftPwm = leftPwm + min(1, max(-1, deltaLeftPwm));    
    int deltaRightPwm = rightPwm-lastRightPwm;
    rightPwm = rightPwm + min(1, max(-1, deltaRightPwm));    
  }
  if (mowDriverChip.usePwmRamp){
    int deltaMowPwm = mowPwm-lastMowPwm;
    mowPwm = mowPwm + min(1, max(-1, deltaMowPwm));      
  }  

  // remember last PWM values
  lastLeftPwm = leftPwm;  
  lastRightPwm = rightPwm;
  lastMowPwm = mowPwm;

  // apply motor PWMs
  setMotorDriver(pinMotorLeftDir, pinMotorLeftPWM, leftPwm, gearsDriverChip, leftSpeedSign);
  setMotorDriver(pinMotorRightDir, pinMotorRightPWM, rightPwm, gearsDriverChip, rightSpeedSign);
  setMotorDriver(pinMotorMowDir, pinMotorMowPWM, mowPwm, mowDriverChip, mowSpeedSign);
  
  // disable driver at zero speed (brake function)    
  bool enableGears = gearsDriverChip.enableActive;
  bool enableMow = mowDriverChip.enableActive;  
  if (gearsDriverChip.disableAtPwmZeroSpeed){  
    if ((leftPwm == 0) && (rightPwm == 0)){
      enableGears = !gearsDriverChip.enableActive;                
    }
    digitalWrite(pinMotorEnable, enableGears);
  }
  if (mowDriverChip.disableAtPwmZeroSpeed){ 
    if (mowPwm == 0) {
      if (mowDriverChip.disableAtPwmZeroSpeed){
        enableMow = !mowDriverChip.enableActive;
      }
    }      
    digitalWrite(pinMotorMowEnable, enableMow);
  }  
}


void AmMotorDriver::getMotorFaults(bool &leftFault, bool &rightFault, bool &mowFault){ 
  return;
  /*
  if (digitalRead(pinMotorLeftFault) == gearsDriverChip.faultActive) {
    leftFault = true;
  }
  if  (digitalRead(pinMotorRightFault) == gearsDriverChip.faultActive) {
    rightFault = true;
  }
  if (digitalRead(pinMotorMowFault) == mowDriverChip.faultActive) {
    mowFault = true;
  }
  */
}

void AmMotorDriver::resetMotorFaults(){ 
  return;
  /* 
  if (digitalRead(pinMotorLeftFault) == gearsDriverChip.faultActive) {
    if (gearsDriverChip.resetFaultByToggleEnable){
      digitalWrite(pinMotorEnable, !gearsDriverChip.enableActive);
      digitalWrite(pinMotorEnable, gearsDriverChip.enableActive);
    }
  }
  if  (digitalRead(pinMotorRightFault) == gearsDriverChip.faultActive) {
    if (gearsDriverChip.resetFaultByToggleEnable){
      digitalWrite(pinMotorEnable, !gearsDriverChip.enableActive);
      digitalWrite(pinMotorEnable, gearsDriverChip.enableActive);
    }
  }
  if (digitalRead(pinMotorMowFault) == mowDriverChip.faultActive) {
    if (mowDriverChip.resetFaultByToggleEnable){
      digitalWrite(pinMotorMowEnable, !mowDriverChip.enableActive);
      digitalWrite(pinMotorMowEnable, mowDriverChip.enableActive);
    }
  }
  */
}

void AmMotorDriver::getMotorCurrent(float &leftCurrent, float &rightCurrent, float &mowCurrent){
  // current (amps)= ((ADCvoltage + ofs)^pow) * scale

      Center_Mow_Current = CenterMowIna226.readShuntCurrent() ;
	    Left_Mow_Current = LeftMowIna226.readShuntCurrent() ;
	    Right_Mow_Current = RightMowIna226.readShuntCurrent() ;
  	  float motorMowCurrent = max(Center_Mow_Current, Left_Mow_Current);
	    mowCurrent = max(motorMowCurrent, Right_Mow_Current);	        
/*

  float ValuePosCheck	= 0;
  ValuePosCheck = (((float)ADC2voltage(analogRead(pinMotorLeftSense))) + gearsDriverChip.adcVoltToAmpOfs);
  if (ValuePosCheck < 0) ValuePosCheck = 0;	// avoid negativ numbers
  leftCurrent = pow(
      ValuePosCheck, gearsDriverChip.adcVoltToAmpPow
      )  * gearsDriverChip.adcVoltToAmpScale;

  ValuePosCheck = (((float)ADC2voltage(analogRead(pinMotorRightSense))) + gearsDriverChip.adcVoltToAmpOfs);
  if (ValuePosCheck < 0) ValuePosCheck = 0;	// avoid negativ numbers
  rightCurrent = pow(
      ValuePosCheck, gearsDriverChip.adcVoltToAmpPow
      )  * gearsDriverChip.adcVoltToAmpScale;

  ValuePosCheck = (((float)ADC2voltage(analogRead(pinMotorMowSense))) + gearsDriverChip.adcVoltToAmpOfs);
  if (ValuePosCheck < 0) ValuePosCheck = 0;	// avoid negativ numbers
  mowCurrent = pow(
            ValuePosCheck, mowDriverChip.adcVoltToAmpPow
      )  * mowDriverChip.adcVoltToAmpScale;
      */
}

void AmMotorDriver::getMotorEncoderTicks(int &leftTicks, int &rightTicks, int &mowTicks){
  leftTicks = odomTicksLeft;
  rightTicks = odomTicksRight;  
  mowTicks = odomTicksMow;
  // reset counters
  odomTicksLeft = odomTicksRight = odomTicksMow = 0;
}    




// ------------------------------------------------------------------------------------


// --- battery switch off circuit --------------------
// JP8 Dauer-ON : automatic switch off circuit disabled
// JP8 Autom.   : automatic switch off circuit enabled
// Note: to increase hardware switch-off time increase capacitor C12  (under DC/DC module)


void AmBatteryDriver::begin(){
  // keep battery switched ON
  pinMode(pinBatterySwitch, OUTPUT);    
  digitalWrite(pinBatterySwitch, HIGH);  
  batteryFactor = (100+10) / 10;    // ADC voltage to battery voltage

  //INA169:  A precision amplifier measures the voltage across the Rs=0.1 ohm, 1% sense resistor. 
  //The Rs is rated for 2W continuous so you can measure up to +5A continuous. 
  //The output is a current that is drawn through the on-board RL=10K+10K=20K resistors so that the 
  // output voltage is 2V per Amp. So for 1A draw, the output will be 2V. You can change the 
  // load resistor RL to be smaller by soldering the bridge If you solder the bridge (RL=10K resistor) 
  // you'll get 1V per Amp.   
  //
  // Is = Vout * 1k / (Rs * RL)

  // PCB1.3 (external INA module)
  //   a) bridged      RL=10K:    Is = 1V * 1k / (0.1*10K)  = 1A
  //   b) non-bridged  RL=20k:    Is = 1V * 1k / (0.1*20K)  = 0.5A
  // PCB1.4 (INA soldered on main PCB)
  //   a) bridged      RL=6.8K:   Is = 1V * 1k / (0.05*6.8K)  = 2.941A
  //   b) non-bridged  RL=10.1k:  Is = 1V * 1k / (0.05*10.1K)  = 1.98A
  
  currentFactor = CURRENT_FACTOR;         // ADC voltage to current ampere  (0.5 for non-bridged)

  pinMode(pinChargeRelay, OUTPUT);
 // pinMode(pinBatteryVoltage, INPUT);
 // pinMode(pinChargeVoltage, INPUT);
 // pinMode(pinChargeCurrent, INPUT);  
  myHumidity.begin();  
//bber1 
	
		CONSOLE.println ("Starting all the Ina226 current mow motor ");
		CenterMowIna226.begin(0x41);
		LeftMowIna226.begin(0x40);
		RightMowIna226.begin(0x44);

		// Configure INA226
		CenterMowIna226.configure(INA226_AVERAGES_4, INA226_BUS_CONV_TIME_1100US, INA226_SHUNT_CONV_TIME_1100US, INA226_MODE_SHUNT_BUS_CONT);
		LeftMowIna226.configure(INA226_AVERAGES_4, INA226_BUS_CONV_TIME_1100US, INA226_SHUNT_CONV_TIME_1100US, INA226_MODE_SHUNT_BUS_CONT);
		RightMowIna226.configure(INA226_AVERAGES_4, INA226_BUS_CONV_TIME_1100US, INA226_SHUNT_CONV_TIME_1100US, INA226_MODE_SHUNT_BUS_CONT);

		// Calibrate INA226. Rshunt = 0.01 ohm, Max excepted current = 4A
		CenterMowIna226.calibrate(0.01, 4);
		LeftMowIna226.calibrate(0.01, 4);
		RightMowIna226.calibrate(0.01, 4);
  


}


void AmBatteryDriver::run(){
}

    
float AmBatteryDriver::getBatteryVoltage(){
  float voltage;
  float D5VoltageDrop=0.8; // drop voltage on diode
  float batvolt = MotRightIna226.readBusVoltage() ;
  batvolt = batvolt + D5VoltageDrop;
  double accel = 0.05;  //filter percent
  if (abs(voltage - batvolt) > 8)   voltage = batvolt; else voltage = (1.0 - accel) * voltage + accel * batvolt;
  return voltage;
}

float AmBatteryDriver::getChargeVoltage(){
  float voltage;
  float chgvolt = ChargeIna226.readBusVoltage() ;
  double accel = 0.05;  //filter percent
  if (abs(voltage - chgvolt) > 8)   voltage = chgvolt; else voltage = (1.0 - accel) * voltage + accel * chgvolt;
  return voltage;
}


float AmBatteryDriver::getChargeCurrent(){ 
    float amps;
    float curramp = ChargeIna226.readBusPower(); //  ?? sense don't work so read power and divise by voltage
    if (getChargeVoltage() != 0) {
      curramp = curramp / getChargeVoltage();
    }
    else
    {
      curramp = 0;
    }

    double accel = 0.05;  //filter percent
    if (abs(amps - curramp) > 0.4) amps = curramp; else amps = (1.0 - accel) * amps + accel * curramp; //Deaktiviert fÃ¼r Ladestromsensor berechnung
  
    return amps;
}

float AmBatteryDriver::getBatteryTemperature(){
  #ifdef USE_TEMP_SENSOR
    // https://learn.sparkfun.com/tutorials/htu21d-humidity-sensor-hookup-guide
    return(myHumidity.readTemperature());
    //float humidity = myHumidity.readHumidity();                  
  #else
    return 0;
  #endif
}

void AmBatteryDriver::enableCharging(bool flag){
  digitalWrite(pinChargeRelay, flag);      
}

void AmBatteryDriver::keepPowerOn(bool flag){
  digitalWrite(pinBatterySwitch, flag);
}


// ------------------------------------------------------------------------------------
void BumperLeftInterruptRoutine(){
  leftPressed = (digitalRead(pinBumperLeft) == LOW);  
}

void BumperRightInterruptRoutine(){
  rightPressed = (digitalRead(pinBumperRight) == LOW);  
}


void AmBumperDriver::begin(){	
  pinMode(pinBumperLeft, INPUT_PULLUP);                   
  pinMode(pinBumperRight, INPUT_PULLUP);                   
  attachInterrupt(pinBumperLeft, BumperLeftInterruptRoutine, CHANGE);
	attachInterrupt(pinBumperRight, BumperRightInterruptRoutine, CHANGE);
}

void AmBumperDriver::getTriggeredBumper(bool &leftBumper, bool &rightBumper){
  leftBumper = leftPressed;
  rightBumper = rightPressed;
}

bool AmBumperDriver::obstacle(){
  return (leftPressed || rightPressed);
}
    
bool AmBumperDriver::getLeftBumper(){
  return (leftPressed);
}

bool AmBumperDriver::getRightBumper(){
  return (rightPressed);
}

void AmBumperDriver::run(){  
}


// ------------------------------------------------------------------------------------


void AmStopButtonDriver::begin(){
  nextControlTime = 0;
  pressed = false;  
  pinMode(pinButton, INPUT_PULLUP);  
}

void AmStopButtonDriver::run(){
  unsigned long t = millis();
  if (t < nextControlTime) return;
  nextControlTime = t + 100;                                       // save CPU resources by running at 10 Hz
  pressed = (digitalRead(pinButton)== LOW);
}

bool AmStopButtonDriver::triggered(){
  return pressed;
}


// ------------------------------------------------------------------------------------


void AmRainSensorDriver::begin(){
  nextControlTime = 0;
  isRaining = false;  
  pinMode(pinRain, INPUT);
}

void AmRainSensorDriver::run(){
  unsigned long t = millis();
  if (t < nextControlTime) return;
  nextControlTime = t + 100;                                       // save CPU resources by running at 10 Hz
  isRaining = (digitalRead(pinRain)== LOW);
}

bool AmRainSensorDriver::triggered(){
  return isRaining;
}

// ------------------------------------------------------------------------------------


void AmLiftSensorDriver::begin(){
  nextControlTime = 0;
  isLifted = false;  
}

void AmLiftSensorDriver::run(){
  unsigned long t = millis();
  if (t < nextControlTime) return;
  nextControlTime = t + 100;                                       // save CPU resources by running at 10 Hz
  //isLifted = (digitalRead(pinLift)== LOW);
}

bool AmLiftSensorDriver::triggered(){
  return isLifted;
}

// ------------------------------------------------------------------------------------

void AmBuzzerDriver::begin(){  
  pinMode(pinBuzzer, OUTPUT);                
  digitalWrite(pinBuzzer, LOW);
}

void AmBuzzerDriver::run(){  
}

void AmBuzzerDriver::noTone(){  
  #ifdef _SAM3XA_
    Timer1.stop();  
    digitalWrite(pinBuzzer, LOW);
  #elif __SAMD51__  // __SAMD51__
    //::noTone(pinBuzzer);     
    zerotimer.enable(false);
    digitalWrite(pinBuzzer, LOW);
  #endif     
}

void AmBuzzerDriver::tone(int freq){  
  #ifdef _SAM3XA_
    pinMode(pinBuzzer, OUTPUT);
    Timer1.attachInterrupt(toneHandler).setFrequency(freq).start();   
  #elif __SAMD51__      // __SAMD51__
    //::tone(pinBuzzer, freq);    

    // Set up the flexible divider/compare
    uint8_t divider  = 1;
    uint16_t compare = 0;
    tc_clock_prescaler prescaler = TC_CLOCK_PRESCALER_DIV1;
    
    divider = 16;
    prescaler = TC_CLOCK_PRESCALER_DIV16;
    compare = (48000000/16)/freq;   
    
    zerotimer.enable(false);
    zerotimer.configure(prescaler,       // prescaler
            TC_COUNTER_SIZE_16BIT,       // bit width of timer/counter
            TC_WAVE_GENERATION_MATCH_PWM // frequency or PWM mode
            );

    zerotimer.setCompare(0, compare);
    zerotimer.setCallback(true, TC_CALLBACK_CC_CHANNEL0, toneHandler);
    zerotimer.enable(true);
  #endif     
}

#endif  // #ifndef __linux__
