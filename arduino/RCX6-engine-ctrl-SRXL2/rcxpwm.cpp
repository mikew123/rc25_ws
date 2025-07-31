/*****************************************************************************************
rcxpwm.cpp

Code for PWM pins on the RCX6 engine controller

The PWM outputs use the 107-Arduino-Servo-RP2040 library using PIO
PWM mux selection PWM output muxsel on PIN_SEL
Steering PWM output on PIN_sSteer
Shift PWM output on PIN_sShift

The PWM inputs use pin interrupts and the millis() timer
Steering PWM input on PIN_mSteer
Shift PWM input on PIN_mShift

Mike Williamson 7/2/2025
*****************************************************************************************/


#include "rcxpwm.h"


// computer signals to servos and motor
void RCXPWM::loopCode() {
  int muxSelPwm;
  int shiftGearPwm;
  int steerPwm;

  if(mode == "passthru") {
    // Decode steering and shift signals from the receiver
    if(mSteer_meas_rdy == 1) {
      if(mSteer_wid>steerCenter) {
        sSteerPct = (100.0*(mSteer_wid-steerCenter))/(steerRightMax-steerCenter);
      } else if(mSteer_wid<steerCenter) {
        sSteerPct = -(100.0*(steerCenter-mSteer_wid)/(steerCenter-steerLeftMax));
      } else {
        sSteerPct = 0;
      }
    } else {
        sSteerPct = 0;
    }

    if(mShift_meas_rdy == 1){
      // Decode shift PWM signal from receiver
      if(mShift_wid >= shiftHigh*0.8) sShiftGear = "high";
      if(mShift_wid <= shiftLow*1.2) sShiftGear = "low";
    } else {
      sShiftGear = "low";
    }
  } 

  // Generate PWM signals for PWM MUX
  if ((mode=="passthru") || ((mode=="term") && (failsafeActive==false))) {
    // send decoded or generated signals to pwm mux
    // Convert steering percent to PWM widths
    int steerPwmOffset = 0;
    if(sSteerPct>0) {
      steerPwmOffset = (+sSteerPct/100.0)*(steerRightMax - steerCenter);
    } else {
      steerPwmOffset = (-sSteerPct/100.0)*(steerLeftMax - steerCenter);
    }
    steerPwm = steerCenter + steerPwmOffset;

    // Convert Shift string to PWM widths
    if(sShiftGear=="high") {
      shiftGearPwm = shiftHigh;
    } else {
      shiftGearPwm = shiftLow;
    }
    muxSelPwm = SEL_COMP;
  } else {
    steerPwm = steerCenter;
    shiftGearPwm = shiftLow; // Should we really change gears?
    muxSelPwm = SEL_RCVR;
  }

// Serial.print("steerPwm = ");
// Serial.println(steerPwm);

  // send signals to pwm mux
  sSteer.writeMicroseconds(steerPwm);
  sShift.writeMicroseconds(shiftGearPwm);
  muxSel.writeMicroseconds(muxSelPwm);
}


void RCXPWM::setSteerPct(float steerPct){
  sSteerPct = steerPct;
Serial.print("set steerPct = ");
Serial.println(sSteerPct);
}

void RCXPWM::setShiftGear(String shiftGear){
  sShiftGear = shiftGear;
}

float RCXPWM::getSteerPct(){
  return sSteerPct;
}

String RCXPWM::getShiftGear(){
  return sShiftGear;
}


void RCXPWM::setMode(String m) {
  mode = m;
}

void RCXPWM::setFailsafeActive(bool failsafe) {
  failsafeActive = failsafe;
}


void RCXPWM::configPWM() {
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
  sShift.writeMicroseconds(shiftLow);

  // PWM inputs decode for shift and steer signals from receiver
  pinMode(PIN_mSteer, INPUT_PULLUP);
  pinMode(PIN_mShift, INPUT_PULLUP);
  // attachInterrupt(digitalPinToInterrupt(PIN_mSteer), pin_mSteer_interrupt, CHANGE) ;
  // attachInterrupt(digitalPinToInterrupt(PIN_mShift), pin_mShift_interrupt, CHANGE) ;
}



/**************************************************************************************
 * INTERRUPTS (instanced in main)
 **************************************************************************************/

// TODO: Handle wrap??
void RCXPWM::pin_mSteer_interruptX (void) {
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

void RCXPWM::pin_mShift_interruptX (void) {
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


