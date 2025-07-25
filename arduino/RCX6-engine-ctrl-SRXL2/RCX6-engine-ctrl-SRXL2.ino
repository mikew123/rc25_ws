/* *************************************

************************************* */

#include <Arduino_JSON.h>
#include <strings.h>


#include "RPi_Pico_TimerInterrupt.h"
RPI_PICO_Timer ITimer(0);
#define TIMER_PER_USEC 100


/**************************************************************************************
 * ODOM encoder
 * This needs to be moved to its own library class
 **************************************************************************************/

#include "pio_encoder.h"
#define PIN_SHAFT_ODOM_A 14
#define PIN_SHAFT_ODOM_B 15
#define ODOM_PER_MSEC 10 /* 10 msec, 100 Hz */
PioEncoder shaftEncoder = {PIN_SHAFT_ODOM_A};

// ODOM POD
#define ODOM_RADUIS_M (0.048/2)
#define ODOM_CIRCUMFERENCE_M (2*PI*ODOM_RADUIS_M)
#define ODOM_ENCODER_COUNT_PER_ROTATION (2000.0)

uint32_t odomTimerLastMs = 0;

int encoderPolarity = 1;
int32_t currEncoderCount = 0;

int32_t lastEncoderCount = 0;
int16_t diffEncoderCount = 0;
double currTravelMeters = 0;
double diffTravelMeters = 0;
double velocityMPS = 0;
uint32_t odomCurrTimeMs = 0;
uint32_t odomLastTimeMs = 0;
uint32_t odomDiffTimeMs = 0;



void ResetEncoderValues() {
  odomCurrTimeMs = millis();
  odomLastTimeMs = odomCurrTimeMs;
  odomDiffTimeMs = 0;

  currEncoderCount = 0;


  lastEncoderCount = 0;
  diffEncoderCount = 0;
  currTravelMeters = 0;
  diffTravelMeters = 0;
  velocityMPS = 0;
}

void InitEncoders() {
  shaftEncoder.begin();
  shaftEncoder.reset();

  ResetEncoderValues();
}

// This is called from an interrupt handler - be quick!
// TODO: should it have its own interrupt?
void OdomHandler() {


  odomCurrTimeMs = millis();

  if(odomCurrTimeMs < (odomLastTimeMs + ODOM_PER_MSEC)) return;

  odomDiffTimeMs = odomCurrTimeMs - odomLastTimeMs;
  odomLastTimeMs = odomCurrTimeMs;

  // read pod encoder
  currEncoderCount = shaftEncoder.getCount() * encoderPolarity;


  // Process pod encoder values 
  diffEncoderCount = currEncoderCount - lastEncoderCount;
  lastEncoderCount = currEncoderCount;
//Serial.print(diffEncoderCount);Serial.print(":");
  // currTravelMeters = (currEncoderCount/ODOM_ENCODER_COUNT_PER_ROTATION)*ODOM_CIRCUMFERENCE_M;
  // diffTravelMeters = (diffEncoderCount/ODOM_ENCODER_COUNT_PER_ROTATION)*ODOM_CIRCUMFERENCE_M;
  // velocityMPS = diffTravelMeters/(1e-6*diffTimeUs);


  // // PID loop to regulate R and L wheel speed based on wheel encoder values
  // for(int enc=0; enc<2; enc++) {
  //   // Limit acceleration and decelleration of wheel velocity
  //   float targetMps = WheelMps[enc];
  //   float limitedMps = WheelMpsAccLimited[enc];
  //   float dMpsAcc = wheelAccellerationMaxMpsPs * (diffTimeUs/1e6);

  //   if(limitedMps != targetMps) {
  //     if(limitedMps < targetMps) {
  //       limitedMps = limitedMps + dMpsAcc;
  //       if(limitedMps > targetMps) limitedMps = targetMps;
  //     } else if(limitedMps > targetMps) {
  //       limitedMps = limitedMps - dMpsAcc;
  //       if(limitedMps < targetMps) limitedMps = targetMps;
  //     } 
  //     WheelMpsAccLimited[enc] = limitedMps;
  //   }

  //   // error is positive when measured velocity > target velocity
  //   PidErrorMps[enc] = velocityMPS[enc] - limitedMps;
  //   PidPropPct[enc] = PidErrorMps[enc] * PidPropCoef[enc];
  //   PidIntPct[enc] += PidErrorMps[enc] * PidIntCoef[enc];
  //   if(PidIntPct[enc] > +PidIntPctMax) PidIntPct[enc] = +PidIntPctMax;
  //   if(PidIntPct[enc] < -PidIntPctMax) PidIntPct[enc] = -PidIntPctMax;

  //   float pct = 100*(PidPropPct[enc] + PidIntPct[enc]);
  //   // TODO: limit pct here? or count on limit check in MotorDrivePct()
  //   MotorPct[enc] = pct;
  // }

  // // Set wheel motor drive PWM percent
  // MotorDrivePct(MotorPct[1], MotorPct[0]);

}

/**************************************************************************************
 * 
 **************************************************************************************/


// local files in arduino sketch folder
#include "srxl2.h"
SRXL2 srx;

#include "rcxpwm.h"
RCXPWM pwm;

// TODO: Handle wrap??
// Decode gear shift PWM signal from receiver using pin edge interrupts and timer
void pin_mShift_interrupt (void) {
  pwm.pin_mShift_interruptX();
}
// Decode steering PWM signal from receiver using pin edge interrupts and timer
void pin_mSteer_interrupt (void) {
  pwm.pin_mSteer_interruptX();
}
void attachInterrupts() {
  attachInterrupt(digitalPinToInterrupt(PIN_mSteer), pin_mSteer_interrupt, CHANGE) ;
  attachInterrupt(digitalPinToInterrupt(PIN_mShift), pin_mShift_interrupt, CHANGE) ;
}

/**************************************************************************************
 * GLOBAL VARIABLES
 **************************************************************************************/

// These values are not being decoded yet
bool failsafeActive = false;
bool receiverSignalsValid = true;

float sThrottlePct = 0.0;
int sSteerPct = 0; 
String sShiftGear = "low";

// Loop timer for precise interval periods
unsigned long statusMillis_last = 0;

String mode_g = "bypass";
int loopStatusPeriod = 100;//1000;


/***************************************
* Hardware PICO TIMER 
*******************************************/

// Configure the timer for the SRXL2, used to time TXEN signals
// Also used for Odom wheel encoder
void configTimer() {
  srx.configTimerTickIntervalUsec(TIMER_PER_USEC);
  if (ITimer.attachInterruptInterval(TIMER_PER_USEC, timerTick))
    {Serial.print(F("Starting ITimer OK, period usec = ")); Serial.println(TIMER_PER_USEC);}
  else
    {Serial.println(F("Can't set ITimer. Select another freq. or timer"));}
  Serial.flush();  
}

// Tick the timer in SRXL2 and execute Odom Wheel encoder
bool timerTick(struct repeating_timer *t) { 
  (void) t;
  srx.timerTick();
  OdomHandler();
  watchdogTimer();
  return true;
}

/********************************************************
* Ackerman conversion
* Convert linear and angular velocity to steering and throttle percent
*********************************************************/
float wheelCircumference = 750.0; // mm
float odomCountPerWheelRotation = 3201.6;
void AckermanConvert(float linX, float angZ) {
  // Ackerman steering conversion
  // linear X is in m/s, angular Z is in rad/s  
  // From test plots, need to determine a good formula from measured data
  // mps = about 0 at thrPct = 20;
  // mps = about 0.8 at thrPct = 100;
  float pctPerFwdMps = 90.58;
  float pctPerRvsMps = 106.05;
  // TODO: PWM for lower speeds?
  if(linX==0.0) sThrottlePct = 0;
  else if(linX>0.0) {
    sThrottlePct = 20 + (linX * pctPerFwdMps);
    if(sThrottlePct > 100) sThrottlePct = 100;
    if(sThrottlePct < 21) sThrottlePct = 0;
  } else {
    sThrottlePct = -20 + (linX * pctPerRvsMps);
    if(sThrottlePct < -100) sThrottlePct = -100;
    if(sThrottlePct > -21) sThrottlePct = 0;
  }
  Serial.print("throttle = ");
  Serial.println(sThrottlePct);
  srx.setThrottlePct(sThrottlePct);
  
  // TODO: steering conversion from angular velocity
}

/********************************************************
* Watchdog timer
* Called from timer "Tick" interrupt
*********************************************************/
uint32_t wdTimeStopMsec = 0; // watchdog time in msec, 0=disabled
uint32_t wdTimeMsec = 0;
// Called from interrupt 
void watchdogTimer() {
  if (wdTimeStopMsec > 0) {
    if (millis() > wdTimeStopMsec) {
      // watchdog timeout
      wdTimeStopMsec = 0; // disable watchdog until new command
      Serial.println("Watchdog timeout, stopping motor");
      sThrottlePct = 0;
      sSteerPct = 0;
      srx.setThrottlePct(sThrottlePct);
      pwm.setSteerPct(sSteerPct);
    }
  }
}

// Call at each valid command to reset the watchdog timer
void resetWatchdogTimer() {
  if (wdTimeMsec > 0) {
    wdTimeStopMsec = millis() + wdTimeMsec;
  } else {
    wdTimeStopMsec = 0; // disable watchdog
  }
}

/********************************************************
* USB serial JSON code
*********************************************************/
// retrieve JSON messages from USB serial port
void getJsonMsgs() {
  if (Serial.available() > 0) {
    // read the incoming string and parse it
    String incomingString = Serial.readStringUntil('\n');
    jsonParse(incomingString.c_str());
  }
}

bool jsonParse(const char *jsonStr) {
  //Serial.println(jsonStr);

  JSONVar myObject = JSON.parse(jsonStr);

//  if (JSON.typeof(myObject) == "undefined") {
  if (JSON.typeof(myObject) != "object") {
    Serial.println("Parsing JSON string input failed! Not a valid object");
    return false;
  }

  resetWatchdogTimer();

  // mode bypass/passthru/term
  if (myObject.hasOwnProperty("mode")) {
    mode_g = (String) myObject["mode"];
    Serial.print("mode = ");
    Serial.println(mode_g);
    
    configureMode(); //TODO: move to srx.setMode
    srx.setMode(mode_g);
    pwm.setMode(mode_g);
  }

  // Watchdog time integer milli-seconds, 0=disabled
  // any command will reset the watchdog timer
  // If watchdog times out then the motor stops
  if (myObject.hasOwnProperty("wd")) {
    wdTimeMsec = (int) myObject["wd"];
    Serial.print("wdTimeMsec = ");
    Serial.println(wdTimeMsec);
    resetWatchdogTimer();
  }

  // Command Velocity array [linear_X, angular_Z]
  if (myObject.hasOwnProperty("cv")) {
    JSONVar cv;
    cv = myObject["cv"];
    Serial.print("CmdVar = [lx=");
    Serial.print((double)cv[0]);
    Serial.print(", az=");
    Serial.print((double)cv[1]);
    Serial.println("]");

    float linX = (double)cv[0];
    float angZ = (double)cv[1];
    AckermanConvert(linX, angZ);
  }

  // Steering percent
  if (myObject.hasOwnProperty("str")) {
    sSteerPct = (int) myObject["str"];
    Serial.print("steer = ");
    Serial.println(sSteerPct);
    pwm.setSteerPct(sSteerPct);
    srx.setSteerPct(sSteerPct);
  }

  // Throttle percent
  if (myObject.hasOwnProperty("thr")) {
    sThrottlePct = (double) myObject["thr"];
    Serial.print("throttle = ");
    Serial.println(sThrottlePct);
    srx.setThrottlePct(sThrottlePct);
  }

  // Shift gear high/low
  if (myObject.hasOwnProperty("sft")) {
    sShiftGear = (String) myObject["sft"];
    Serial.print("gear = ");
    Serial.println(sShiftGear);
    pwm.setShiftGear(sShiftGear=="high"?"high":"low");
    srx.setShiftGear(sShiftGear=="high"?"high":"low");
  }

  return true;
}

//TODO: move to SRXL2
void configureMode(){
  if(mode_g == "bypass") {
    digitalWrite(BYPASS, 0); // enable bypass switch 
  }
  else if(mode_g == "passthru" || mode_g == "term") {
    digitalWrite(BYPASS, 1); // disable bypass switch
  }
  // to be safe disable SRXL2 serial transmits
  srx.disableTX();
}

// USB Serial
void configSerial(){
  Serial.begin(1000000);
  while(!Serial) delay(100);
  Serial.println("USB Serial started");
}


// Impliment failsafe
// When computer is selected the transmitter trigger must be pulled to operate
// If the trigger is not pulled then the failsafe is activated and the signals
// to the servos and motor become the default values

// Monitor failsafe mechanism
// DEBUG: disabled for now
// void checkFailsafe() {
  // if (muxSelRcvr == true) {
    // failsafeActive = false;
  // } else  {
    // computer is selected for control
    // if (mThrottle_wid < failsafeThrottleThresh) {
    //   // Failsafe active when throttle on transmitter is released
    //   failsafeActive = true;
    // } else {
    //   failsafeActive = false;
    // }
  // }

  // pwm.setFailsafeActive(failsafeActive);
//}

// Send status message to host computer
void sendStatusMsg() {

  JSONVar myObject;

  // myObject["gear"] = pwm.getShiftGear();

  // myObject["mode"] = mode_g;

  // myObject["fsa"] = failsafeActive;

  // myObject["rca"] = receiverSignalsValid;


  myObject["millis"] = millis();

  myObject["ocurr"] = currEncoderCount;

  myObject["odiff"] = diffEncoderCount;

  // get motor RPM from telemetry
  int motorRpm = srx.getEscRpm();
  myObject["rpm"] = motorRpm; 

  String jsonString = JSON.stringify(myObject);
  Serial.println(jsonString);

}

//void checkTransmitterActive() {
  // TODO: Use throttle serial activity???

  // TODO: This does not work after transmitter 1st powered on
  //       Maybe monitor period variation which seems small when transmitter
  //       is powered on. But probably track an average nominal instead of
  //       fixed nominal to compensate for temp and age etc
  //       Also check receiver signal timeout 
  // check for active valid receiver signals
  // verify that PWM period is within 10%
  // float mSteerPctOffset = 100.0*(mSteer_per-pwmPerNominal)/pwmPerNominal;
  //Serial.println(mSteerPctOffset);
  // if (   (fabs(mSteerPctOffset) < 10)) {
  //  receiverSignalsValid = true;
  // } else {
  //   receiverSignalsValid = false;
  // }
//}


// Select receiver or computer using steering and shift and throttle
// The throttle must me center +- 5% to change selection
// Use shift high >1500+10%, low <1500-10%
// Select computer when steering > center+10% and shift high to low
// Select receiver when steering < center-10% and shift high to low
//  void muxSelectDecode() {
  //  if(fabs(mThrottle_wid-throttleZero) < (throttleZero*0.1)) {
  //   if(mShift_wid < 1500*0.9) {
  //     if(shiftState==1) {
  //       if(mSteer_wid > steerCenter*1.05) muxSelRcvr = true;
  //       else muxSelRcvr = false;
  //     }
  //     shiftState = 0;
  //   }
  //   if(mShift_wid > 1500*1.1) {
  //     shiftState = 1;
  //   }
  //}

//  }

void pwmLoopCode() {
  pwm.loopCode();
}

void srxLoopCode() {
  srx.loopCode();
}

// Send status 1/sec
void sendStatus(unsigned long loopMillis) {
  if ((loopMillis - statusMillis_last) >= loopStatusPeriod) {
    sendStatusMsg();
    statusMillis_last = loopMillis;
  }
}


/****************************************************/
void setup() {
  srx.configPins();
  configSerial();
  srx.configSerialESC();
  srx.configSerialRCV();
  configTimer();
  pwm.configPWM();
  attachInterrupts();
  InitEncoders();


}

/****************************************************/
void loop() {
  unsigned long loopMillis = millis();

  // impliment failsafe mechanism
  // checkFailsafe();

  // Decode mux selection from receiver signals
  // muxSelectDecode();

  getJsonMsgs();

  srxLoopCode();

  pwmLoopCode();

  sendStatus(loopMillis);

}
