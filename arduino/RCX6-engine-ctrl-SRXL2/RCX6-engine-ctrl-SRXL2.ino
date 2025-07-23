/* *************************************

************************************* */

#include <Arduino_JSON.h>
#include <strings.h>


#include "RPi_Pico_TimerInterrupt.h"
RPI_PICO_Timer ITimer(0);
#define TIMER_PER_USEC 100


#include "pio_encoder.h"
#define PIN_SHAFT_ODOM_A 14
#define PIN_SHAFT_ODOM_B 15
#define ODOM_PER_MSEC 100 /* 10 msec, 100 Hz */
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
Serial.print(diffEncoderCount);Serial.print(":");
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

int sThrottlePct = 0;
int sSteerPct = 0; 
String sShiftGear = "low";

// Loop timer for precise interval periods
unsigned long statusMillis_last = 0;

String mode_g = "bypass";
int loopStatusPeriod = 1000;


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
  return true;
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

  if (JSON.typeof(myObject) == "undefined") {
    Serial.println("Parsing JSON string input failed!");
    return false;
  }


  if (myObject.hasOwnProperty("mode")) {
    mode_g = (String) myObject["mode"];
    Serial.print("mode = ");
    Serial.println(mode_g);
    
    configureMode(); //TODO: move to srx.setMode
    srx.setMode(mode_g);
    pwm.setMode(mode_g);
  }

  if (myObject.hasOwnProperty("str")) {
    sSteerPct = (int) myObject["str"];
    Serial.print("steer = ");
    Serial.println(sSteerPct);
    pwm.setSteerPct(sSteerPct);
    srx.setSteerPct(sSteerPct);
  }

  if (myObject.hasOwnProperty("thr")) {
    sThrottlePct = (int) myObject["thr"];
    Serial.print("throttle = ");
    Serial.println(sThrottlePct);
    srx.setThrottlePct(sThrottlePct);
  }

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

  myObject["gear"] = pwm.getShiftGear();

  myObject["mode"] = mode_g;

  myObject["fsa"] = failsafeActive;

  myObject["rca"] = receiverSignalsValid;

  myObject["odom"] = diffEncoderCount;

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
