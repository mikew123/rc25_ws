/**
 * PWM for servos and Throttle ESC uses this library
 * This software is distributed under the terms of the MIT License.
 * Copyright (c) 2023 LXRobotics.
 * Author: Alexander Entinger <alexander.entinger@lxrobotics.com>
 * Contributors: https://github.com/107-systems/107-Arduino-Servo-RP2040/graphs/contributors.
 
 
 * Mike Williamson 7-5-2024 engine controller
 * Resides in the "engine compartment"
 * Connects to computer using USB serial
 * Selects RC receiver or computer control
 * Generates PWM signals for steering servo and Throttle ESC

 */

/**************************************************************************************
 * INCLUDES
 **************************************************************************************/

#include <107-Arduino-Servo-RP2040.h>
#include <Arduino_JSON.h>
#include <strings.h>

/**************************************************************************************
 * GLOBAL VARIABLES
 **************************************************************************************/

static _107_::Servo sSteer, sThrottle, sShift, pwm_3, muxSel;

#define pwmPerNominal 11000

#define PIN_sSteer 0
#define PIN_sThrottle 1
#define PIN_sShift 2
#define PIN_PWM3 3

#define PIN_SEL  4
#define SEL_RCVR 1000
#define SEL_COMP 2000

#define PIN_mSteer 5
#define PIN_mThrottle 6
#define PIN_mShift 7

// PWM values 
#define steerCenter 1575
#define steerRightMax 2000
#define steerLeftMax 1000

#define throttleZero 1500
#define throttleFwdMax 1900
#define throttleRevMax 1100
// These are impericaly measured, how do the change with temp?
#define throttleFwdMin 1545
#define throttleRevMin 1475

#define shiftLow 1100 // shift is a static value
#define shiftHigh 1900

#define loopStatusPeriod 1000

#define failsafeThrottleThresh (throttleZero+100)

// Loop timer for precise interval periods
unsigned long statusMillis_last = 0;

unsigned long mSteer_micros_last0 = 0, mSteer_micros_last1 = 0;
bool mSteer_meas_rdy = 0;
int mSteer_per=0, mSteer_wid=0;

unsigned long mThrottle_micros_last0 = 0, mThrottle_micros_last1 = 0;
bool mThrottle_meas_rdy = 0;
int mThrottle_per=0, mThrottle_wid=0;

unsigned long mShift_micros_last0 = 0, mShift_micros_last1 = 0;
bool mShift_meas_rdy = 0;
int mShift_per=0, mShift_wid=0;

bool shiftState = 0;
bool rcSelect = true;

bool failSafeEnable = true;
bool motionEnabled = false;
bool receiverSignalsValid = false;


// Computer (slave mux input) signals
int sSteerPct = 0;
int sThrottlePct = 0;
bool sShiftGear = false; // low gear

/**************************************************************************************
 * INTERRUPTS
 **************************************************************************************/

// TODO: Handle wrap??
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

// TODO: Handle wrap??
void pin_mThrottle_interrupt (void) {
  unsigned long mThrottle_micros = micros();
  if(digitalRead(PIN_mThrottle)==0) {
    // HI->LO transition
    mThrottle_per = mThrottle_micros - mThrottle_micros_last0;
    mThrottle_wid = mThrottle_micros - mThrottle_micros_last1;
    mThrottle_micros_last0 = mThrottle_micros;
    mThrottle_meas_rdy = 1;
  } else {
    // LO->HI transition
    mThrottle_micros_last1 = mThrottle_micros;
  }
}

// TODO: Handle wrap??
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

/**************************************************************************************
 * SETUP
 **************************************************************************************/

void setup()
{
  Serial.begin(1000000);
  while (!Serial) { }

  // PWM outputs
  sSteer.attach(PIN_sSteer);
  sThrottle.attach(PIN_sThrottle);
  sShift.attach(PIN_sShift);
  pwm_3.attach(PIN_PWM3);
  muxSel.attach(PIN_SEL);
  // Select RC receiver as default
  muxSel.writeMicroseconds(SEL_RCVR);
  // Set default PWM signals on slave signals to RC MUX
  sSteer.writeMicroseconds(steerCenter);
  sThrottle.writeMicroseconds(throttleZero);
  sShift.writeMicroseconds(shiftLow);
  pwm_3.writeMicroseconds(1500);

  // PWM inputs
  pinMode(PIN_mSteer, INPUT_PULLUP);
  pinMode(PIN_mThrottle, INPUT_PULLUP);
  pinMode(PIN_mShift, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(PIN_mSteer), pin_mSteer_interrupt, CHANGE) ;
  attachInterrupt(digitalPinToInterrupt(PIN_mThrottle), pin_mThrottle_interrupt, CHANGE) ;
  attachInterrupt(digitalPinToInterrupt(PIN_mShift), pin_mShift_interrupt, CHANGE) ;

  Serial.println("Engine controller started");
}

/**************************************************************************************
 * LOOP
 **************************************************************************************/
void loop()
{
  unsigned long loopMillis = millis();

  // impliment failsafe mechanism
  checkFailsafe();

  // Check serial port for a JSON message
  if (Serial.available() > 0) {
    // read the incoming string and parse it
    String incomingString = Serial.readStringUntil('\n');
    jsonParse(incomingString.c_str());
  }

  // Decode mux selection
  muxSelectDecode();

  // computer signals to servos and motor
  computerSignals();


  // Send status 1/sec
  if ((loopMillis - statusMillis_last) > loopStatusPeriod) {

    // DEBUG
    // monitorReceiver();

    // send a status message
    sendStatus();

    statusMillis_last = statusMillis_last+1000;
  }

  delay(1); // TODO: delay 0 ???
}


// computer signals to servos and motor

void computerSignals() {
  if ((rcSelect==false) && (motionEnabled==true) ) {
    // send computer signals to pwm mux
    // Convert percent signals to PWM widths
    sSteerPct;
    sThrottlePct;
    int steerPwmOffset = 0;
    if(sSteerPct>0) {
      steerPwmOffset = (+sSteerPct/100.0)*(steerRightMax - steerCenter);
    } else {
      steerPwmOffset = (-sSteerPct/100.0)*(steerLeftMax - steerCenter);
    }
    int steerPwm = steerCenter + steerPwmOffset;
    sSteer.writeMicroseconds(steerPwm);

    int throttlePwmOffset = 0;
    int throttlePwm = throttleZero;
    if(sThrottlePct>0) {
      throttlePwmOffset = (+sThrottlePct/100.0)*(throttleFwdMax - throttleFwdMin);
      throttlePwm = throttleFwdMin + throttlePwmOffset;
    } else {
      throttlePwmOffset = (-sThrottlePct/100.0)*(throttleRevMax - throttleRevMin);
      throttlePwm = throttleRevMin + throttlePwmOffset;
    }
    sThrottle.writeMicroseconds(throttlePwm);

    int shiftGearPwm = shiftLow;
    if(sShiftGear==true) {
      shiftGearPwm = shiftHigh;
    } else {
      shiftGearPwm = shiftLow;
    }
    sShift.writeMicroseconds(shiftGearPwm);

  } else {
    // send idle signals to pwm mux
    sSteer.writeMicroseconds(steerCenter);
    sThrottle.writeMicroseconds(throttleZero);
    sShift.writeMicroseconds(shiftLow);
  }
}


// Impliment failsafe
// When computer is selected the transmitter trigger must be pulled to operate
// If the triggefailSafeEnabler is not pulled then the failsafe is activated and the signals
// to the servos and motor become the default values

// Monitor failsafe mechanism
void checkFailsafe() {
  if (rcSelect == true) {
    // disable motion when computer is first selected 
    motionEnabled = false;
  } else  if (failSafeEnable == true) {
    // computer is selected for control
    if (mThrottle_wid < failsafeThrottleThresh) {
      // Motion disabled when throttle on transmitter is released
      motionEnabled = false;
    } else {
      motionEnabled = true;
    }
  } else {
    // fail safe is disabled, allow motion
    motionEnabled = true;
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

  if(rcSelect) {
    myObject["mux"] = true;
  } else {
    myObject["mux"] = false;
  }

  if(motionEnabled) {
    myObject["ena"] = true;
  } else {
    myObject["ena"] = false;
  }

  if(failSafeEnable) {
    myObject["fse"] = true;
  } else {
    myObject["fse"] = false;
  }


  String jsonString = JSON.stringify(myObject);
  Serial.println(jsonString);

}

void checkTransmitterActive() {

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

  if(mThrottle_meas_rdy == 1){
    Serial.print("mThrottle:");
    Serial.print(mThrottle_per);
    Serial.print(",");
    Serial.println(mThrottle_wid);
    mThrottle_meas_rdy = 0;
  }

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
  if(fabs(mThrottle_wid-throttleZero) < (throttleZero*0.1)) {
    if(mShift_wid < 1500*0.9) {
      if(shiftState==1) {
        if(mSteer_wid > steerCenter*1.05) rcSelect = true;
        else rcSelect = false;
      }
      shiftState = 0;
    }
    if(mShift_wid > 1500*1.1) {
      shiftState = 1;
    }
  }

  // TODO: move to a pwm output module
  muxSel.writeMicroseconds(rcSelect?SEL_RCVR:SEL_COMP);

}

// *************************************************************
// JSON string commands entered into Arduino Serial Monitor
// Enable failsafe - {"fse":true}
// Throttle forward 10% - {"thr":10} 
// Steer right 10% - {"str":10}
// Gear set to low - {"sft":false}
bool jsonParse(const char *jsonStr) {
  Serial.println(jsonStr);

  JSONVar myObject = JSON.parse(jsonStr);

  if (JSON.typeof(myObject) == "undefined") {
    Serial.println("Parsing JSON string input failed!");
    return false;
  }

  if (myObject.hasOwnProperty("rc")) {
    rcSelect = (bool) myObject["rc"];
    Serial.print("RC receiver Select = ");
    Serial.println(rcSelect);
  }

  if (myObject.hasOwnProperty("fse")) {
    failSafeEnable = (bool) myObject["fse"];
    Serial.print("Fail Safe Enabled = ");
    Serial.println(failSafeEnable);
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
    sShiftGear = (bool) myObject["sft"];
    Serial.print("gear = ");
    Serial.println(sShiftGear);
  }

  return true;
}


/////////////////////////////////////////////////////////////////////////
// DEMO code :
const char input[] = "{\"result\":true,\"count\":42,\"foo\":\"bar\"}";

void demoParse() {
  Serial.println("parse");
  Serial.println("=====");

  JSONVar myObject = JSON.parse(input);

  // JSON.typeof(jsonVar) can be used to get the type of the variable
  if (JSON.typeof(myObject) == "undefined") {
    Serial.println("Parsing input failed!");
    return;
  }

  Serial.print("JSON.typeof(myObject) = ");
  Serial.println(JSON.typeof(myObject)); // prints: object

  // myObject.hasOwnProperty(key) checks if the object contains an entry for key
  if (myObject.hasOwnProperty("result")) {
    Serial.print("myObject[\"result\"] = ");

    Serial.println((bool) myObject["result"]);
  }

  if (myObject.hasOwnProperty("count")) {
    Serial.print("myObject[\"count\"] = ");

    Serial.println((int) myObject["count"]);
  }

  if (myObject.hasOwnProperty("count")) {
    Serial.print("myObject[\"count\"] = ");

    Serial.println((double) myObject["count"]);
  }

  if (myObject.hasOwnProperty("foo")) {
    Serial.print("myObject[\"foo\"] = ");

    Serial.println((const char*) myObject["foo"]);
  }

  // JSONVars can be printed using print or println
  Serial.print("myObject = ");
  Serial.println(myObject);

  Serial.println();
}

void demoCreation() {
  Serial.println("creation");
  Serial.println("========");

  JSONVar myObject;

  myObject["hello"] = "world";
  myObject["true"] = true;

  myObject["x1"] = (int) 42;
  myObject["x2"] = (long) 42;
  myObject["x3"] = (unsigned long) 42;

  int x1 = myObject["x1"];
  assert(x1 == 42);
  
  long x2 = myObject["x2"];
  assert(x2 == 42);

  unsigned long x3 = myObject["x3"];
  assert(x3 == 42);

  Serial.print("myObject.keys() = ");
  Serial.println(myObject.keys());

  // JSON.stringify(myVar) can be used to convert the JSONVar to a String
  String jsonString = JSON.stringify(myObject);

  Serial.print("JSON.stringify(myObject) = ");
  Serial.println(jsonString);

  Serial.println();

  // myObject.keys() can be used to get an array of all the keys in the object
  JSONVar keys = myObject.keys();

  for (int i = 0; i < keys.length(); i++) {
    JSONVar value = myObject[keys[i]];

    Serial.print("JSON.typeof(myObject[");
    Serial.print(keys[i]);
    Serial.print("]) = ");
    Serial.println(JSON.typeof(value));

    Serial.print("myObject[");
    Serial.print(keys[i]);
    Serial.print("] = ");
    Serial.println(value);

    Serial.println();
  }

  Serial.println();

  // setting a value to undefined can remove it from the object
  myObject["x"] = undefined;

  // you can also change a value
  myObject["hello"] = "there!";

  Serial.print("myObject = ");
  Serial.println(myObject);
}






