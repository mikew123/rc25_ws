/* *************************************

************************************* */

#include <Arduino_JSON.h>
#include <strings.h>

#include <107-Arduino-Servo-RP2040.h>

#include "RPi_Pico_TimerInterrupt.h"
RPI_PICO_Timer ITimer(0);
#define TIMER_PER_USEC 100


// local files in arduino sketch folder
#include "srxl2.h"
SRXL2 srx;




/**************************************************************************************
 * GLOBAL VARIABLES
 **************************************************************************************/

//static _107_::Servo sSteer, sThrottle, sShift, pwm_3, muxSel;
static _107_::Servo sSteer, sShift, muxSel;

#define pwmPerNominal 11000

#define PIN_sSteer 0
//#define PIN_sThrottle 1
#define PIN_sShift 2
//#define PIN_PWM3 3

#define PIN_SEL  4
#define SEL_RCVR 1000
#define SEL_COMP 2000

#define PIN_mSteer 5
//#define PIN_mThrottle 6
#define PIN_mShift 7

// PWM values 
#define steerCenter 1575
#define steerRightMax 2000
#define steerLeftMax 1000

// #define throttleZero 1500
// #define throttleFwdMax 1900
// #define throttleRevMax 1100
// // These are impericaly measured, how do the change with temp?
// #define throttleFwdMin 1545
// #define throttleRevMin 1475

#define shiftLow 1100 // shift is a static value
#define shiftHigh 1900

#define loopStatusPeriod 1000

// #define failsafeThrottleThresh (throttleZero+100)

// Loop timer for precise interval periods
unsigned long statusMillis_last = 0;

unsigned long mSteer_micros_last0 = 0, mSteer_micros_last1 = 0;
bool mSteer_meas_rdy = 0;
int mSteer_per=0, mSteer_wid=0;

// unsigned long mThrottle_micros_last0 = 0, mThrottle_micros_last1 = 0;
// bool mThrottle_meas_rdy = 0;
// int mThrottle_per=0, mThrottle_wid=0;

unsigned long mShift_micros_last0 = 0, mShift_micros_last1 = 0;
bool mShift_meas_rdy = 0;
int mShift_per=0, mShift_wid=0;

bool shiftState = 0;
bool muxSelRcvr = true;


bool failsafeActive = false;
bool receiverSignalsValid = false;


// Computer (slave mux input) signals
int sSteerPct = 0;
int sThrottlePct = 0; // Use for SRXL2 throttle
String sShiftGear = "low";



String mode_g = "bypass";


/***************************************
* Hardware PICO TIMER 
*******************************************/

// Configure the timer for the SRXL2, used to time TXEN signals
void configTimer() {
  srx.configTimerTickIntervalUsec(TIMER_PER_USEC);
  if (ITimer.attachInterruptInterval(TIMER_PER_USEC, timerTick))
    {Serial.print(F("Starting ITimer OK, period usec = ")); Serial.println(TIMER_PER_USEC);}
  else
    {Serial.println(F("Can't set ITimer. Select another freq. or timer"));}
  Serial.flush();  
}

// Tick the timer in SRXL2 
bool timerTick(struct repeating_timer *t) { 
  (void) t;
  srx.timerTick();
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
    
    configureMode();
    srx.setMode(mode_g);
  }

  if (myObject.hasOwnProperty("str")) {
    sSteerPct = (int) myObject["str"];
    Serial.print("steer = ");
    Serial.println(sSteerPct);
  }

  if (myObject.hasOwnProperty("thr")) {
    sThrottlePct = (int) myObject["thr"];
    Serial.print("throttle = ");
    Serial.println(sThrottlePct);
  }

  if (myObject.hasOwnProperty("sft")) {
    sShiftGear = (String) myObject["sft"];
    Serial.print("gear = ");
    Serial.println(sShiftGear);
  }

  return true;
}

void configureMode(){
  if(mode_g == "bypass") {
    digitalWrite(BYPASS, 0); // enable bypass switch 
  }
  else if(mode_g == "passthru") {
    digitalWrite(BYPASS, 1); // disable bypass switch
  }
  // to be safe disable UART outs
  srx.disableTX();
}

// USB Serial
void configSerial(){
  Serial.begin(1000000);
  while(!Serial) delay(100);
  Serial.println("USB Serial started");
}

/**************************************************************************************
 * INTERRUPTS
 **************************************************************************************/

// TODO: Handle wrap??
// Decode steering PWM signal from receiver using pin edge interrupts and timer
void pin_mSteer_interrupt (void) {
  unsigned long mSteer_micros = micros();
  if(digitalRead(PIN_mSteer)==0) {
    // HI->LO transition
    mSteer_per = mSteer_micros - mSteer_micros_last0;
    mSteer_wid = mSteer_micros - mSteer_micros_last1;
    mSteer_micros_last0 = mSteer_micros;
    mSteer_meas_rdy = 1;
  } else {
    // LO->HI transition
    mSteer_micros_last1 = mSteer_micros;
  }
}

// // TODO: Handle wrap??
// void pin_mThrottle_interrupt (void) {
//   unsigned long mThrottle_micros = micros();
//   if(digitalRead(PIN_mThrottle)==0) {
//     // HI->LO transition
//     mThrottle_per = mThrottle_micros - mThrottle_micros_last0;
//     mThrottle_wid = mThrottle_micros - mThrottle_micros_last1;
//     mThrottle_micros_last0 = mThrottle_micros;
//     mThrottle_meas_rdy = 1;
//   } else {
//     // LO->HI transition
//     mThrottle_micros_last1 = mThrottle_micros;
//   }
// }

// TODO: Handle wrap??
// Decode gear shift PWM signal from receiver using pin edge interrupts and timer
void pin_mShift_interrupt (void) {
  unsigned long mShift_micros = micros();
  if(digitalRead(PIN_mShift)==0) {
    // HI->LO transition
    mShift_per = mShift_micros - mShift_micros_last0;
    mShift_wid = mShift_micros - mShift_micros_last1;
    mShift_micros_last0 = mShift_micros;
    mShift_meas_rdy = 1;
  } else {
    // LO->HI transition
    mShift_micros_last1 = mShift_micros;
  }
}


/*****************************************************
* Servo PWM code
******************************************************/
// computer signals to servos and motor

void computerSignals() {
  if ((muxSelRcvr==false) && (failsafeActive==false) ) {
    // send computer signals to pwm mux
    // Convert percent signals to PWM widths
    // sSteerPct;
    // sThrottlePct;
    int steerPwmOffset = 0;
    if(sSteerPct>0) {
      steerPwmOffset = (+sSteerPct/100.0)*(steerRightMax - steerCenter);
    } else {
      steerPwmOffset = (-sSteerPct/100.0)*(steerLeftMax - steerCenter);
    }
    int steerPwm = steerCenter + steerPwmOffset;
    sSteer.writeMicroseconds(steerPwm);

    // int throttlePwmOffset = 0;
    // int throttlePwm = throttleZero;
    // if(sThrottlePct>0) {
    //   throttlePwmOffset = (+sThrottlePct/100.0)*(throttleFwdMax - throttleFwdMin);
    //   throttlePwm = throttleFwdMin + throttlePwmOffset;
    // } else {
    //   throttlePwmOffset = (-sThrottlePct/100.0)*(throttleRevMax - throttleRevMin);
    //   throttlePwm = throttleRevMin + throttlePwmOffset;
    // }
    // sThrottle.writeMicroseconds(throttlePwm);

    int shiftGearPwm = shiftLow;
    if(sShiftGear=="high") {
      shiftGearPwm = shiftHigh;
    } else {
      shiftGearPwm = shiftLow;
    }
    sShift.writeMicroseconds(shiftGearPwm);

  } else {
    // send idle signals to pwm mux
    sSteer.writeMicroseconds(steerCenter);
    // sThrottle.writeMicroseconds(throttleZero);
    sShift.writeMicroseconds(shiftLow);
  }
}


// Impliment failsafe
// When computer is selected the transmitter trigger must be pulled to operate
// If the trigger is not pulled then the failsafe is activated and the signals
// to the servos and motor become the default values

// Monitor failsafe mechanism
// DEBUG: disabled for now
void checkFailsafe() {
  if (muxSelRcvr == true) {
    failsafeActive = false;
  } else  {
    // computer is selected for control
    // if (mThrottle_wid < failsafeThrottleThresh) {
    //   // Failsafe active when throttle on transmitter is released
    //   failsafeActive = true;
    // } else {
    //   failsafeActive = false;
    // }
  }
}

// Send status message to host computer
void sendStatus() {

  JSONVar myObject;

  if(shiftState==0) {
    // low gear
    myObject["gear"] = "low";
  } else {
    // high gear
    myObject["gear"] = "hi";
  }

  if(muxSelRcvr) {
    myObject["mux"] = "rcvr";
  } else {
    myObject["mux"] = "comp";
  }

  if(failsafeActive) {
    myObject["fsa"] = true;
  } else {
    myObject["fsa"] = false;
  }

  if(receiverSignalsValid) {
    myObject["rca"] = true;
  } else {
    myObject["rca"] = false;
  }


  String jsonString = JSON.stringify(myObject);
  Serial.println(jsonString);

}

void checkTransmitterActive() {
  // TODO: Use throttle serial activity???

  // TODO: This does not work after transmitter 1st powered on
  //       Maybe monitor period variation which seems small when transmitter
  //       is powered on. But probably track an average nominal instead of
  //       fixed nominal to compensate for temp and age etc
  //       Also check receiver signal timeout 
  // check for active valid receiver signals
  // verify that PWM period is within 10%
  float mSteerPctOffset = 100.0*(mSteer_per-pwmPerNominal)/pwmPerNominal;
  //Serial.println(mSteerPctOffset);
  if (   (fabs(mSteerPctOffset) < 10)) {
    receiverSignalsValid = true;
  } else {
    receiverSignalsValid = false;
  }
}

void monitorReceiver() {

  // TODO: timeout
  // Monitor PWM signals from RC receiver on master MUX inputs
  if(mSteer_meas_rdy == 1){
    Serial.print("mSteer:");
    Serial.print(mSteer_per);
    Serial.print(",");
    Serial.println(mSteer_wid);
    mSteer_meas_rdy = 0;
  }

  // if(mThrottle_meas_rdy == 1){
  //   Serial.print("mThrottle:");
  //   Serial.print(mThrottle_per);
  //   Serial.print(",");
  //   Serial.println(mThrottle_wid);
  //   mThrottle_meas_rdy = 0;
  // }

  if(mShift_meas_rdy == 1){
    Serial.print("mShift:");
    Serial.print(mShift_per);
    Serial.print(",");
    Serial.println(mShift_wid);
    mShift_meas_rdy = 0;

  }
}

// Select receiver or computer using steering and shift and throttle
// The throttle must me center +- 5% to change selection
// Use shift high >1500+10%, low <1500-10%
// Select computer when steering > center+10% and shift high to low
// Select receiver when steering < center-10% and shift high to low
void muxSelectDecode() {
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

}


void configPWM() {
  // PWM outputs
  sSteer.attach(PIN_sSteer);
  // sThrottle.attach(PIN_sThrottle);
  sShift.attach(PIN_sShift);
  // pwm_3.attach(PIN_PWM3);
  muxSel.attach(PIN_SEL);
  // Select RC receiver as default
  muxSel.writeMicroseconds(SEL_RCVR);
  // Set default PWM signals on slave signals to RC MUX
  sSteer.writeMicroseconds(steerCenter);
  // sThrottle.writeMicroseconds(throttleZero);
  sShift.writeMicroseconds(shiftLow);
  // pwm_3.writeMicroseconds(1500);

  // PWM inputs
  pinMode(PIN_mSteer, INPUT_PULLUP);
  // pinMode(PIN_mThrottle, INPUT_PULLUP);
  pinMode(PIN_mShift, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(PIN_mSteer), pin_mSteer_interrupt, CHANGE) ;
  // attachInterrupt(digitalPinToInterrupt(PIN_mThrottle), pin_mThrottle_interrupt, CHANGE) ;
  attachInterrupt(digitalPinToInterrupt(PIN_mShift), pin_mShift_interrupt, CHANGE) ;
}

void pwmLoopCode() {
  // Decode mux selection
  muxSelectDecode();

  // computer signals to servos and motor
  computerSignals();


}

void srxLoopCode() {
  srx.getPacketDataRxEsc();
  srx.packetPassthruRxEsc();
  srx.sendPacketTxRcv();

  // srx.printPacketRxEsc(id);

  srx.getPacketDataRxRcv();
  srx.packetPassthruRxRcv();
  srx.sendPacketTxEsc();
  // srx.printPacketRxRcv(id);
}


/****************************************************/
void setup() {
  srx.configPins();
  configSerial();
  srx.configSerialESC();
  srx.configSerialRCV();
  configTimer();
  configPWM();
}

/****************************************************/
void loop() {
  unsigned long loopMillis = millis();

  // impliment failsafe mechanism
  checkFailsafe();

  getJsonMsgs();

  srxLoopCode();

  pwmLoopCode();

  // Send status 1/sec
  if ((loopMillis - statusMillis_last) >= loopStatusPeriod) {
// Serial.print("pwmLoopCode: ");Serial.print(statusMillis_last);
// Serial.print(" "); Serial.println(loopMillis);
    // DEBUG
    monitorReceiver();
    // send a status message
    sendStatus();
    statusMillis_last = loopMillis;
  }

// TODO: get rid of this crude test code
  if(mode_g != "bypass") {
    muxSelRcvr = false;

    int mSteerPct = 0;
    if(mSteer_wid>steerCenter) {
      mSteerPct = (100L*(mSteer_wid-steerCenter))/(steerRightMax-steerCenter);
    }
    if(mSteer_wid<steerCenter) {
      mSteerPct = -(100L*(steerCenter-mSteer_wid)/(steerCenter-steerLeftMax));
    }
    sSteerPct = mSteerPct;

    if(mShift_wid >= shiftHigh*0.8) sShiftGear = "high";
    if(mShift_wid <= shiftLow*1.2) sShiftGear = "low";

  } else {
    muxSelRcvr = true;
  }

  // TODO: move to a pwm output module
  muxSel.writeMicroseconds(muxSelRcvr?SEL_RCVR:SEL_COMP);


}
