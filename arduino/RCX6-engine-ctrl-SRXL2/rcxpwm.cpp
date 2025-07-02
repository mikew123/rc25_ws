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

// TODO: get rid of this crude test code
void RCXPWM::crudeTestCode() {
  if(mode != "bypass") {
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


void RCXPWM::setMode(String m) {
  mode = m;
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


/*****************************************************
* Servo PWM code
******************************************************/
// computer signals to servos and motor
void RCXPWM::computerSignals() {
//  if ((muxSelRcvr==false) && (failsafeActive==false) ) {
  if ((muxSelRcvr==false)) {
    // send computer signals to pwm mux
    // Convert percent signals to PWM widths
    int steerPwmOffset = 0;
    if(sSteerPct>0) {
      steerPwmOffset = (+sSteerPct/100.0)*(steerRightMax - steerCenter);
    } else {
      steerPwmOffset = (-sSteerPct/100.0)*(steerLeftMax - steerCenter);
    }
    int steerPwm = steerCenter + steerPwmOffset;
    sSteer.writeMicroseconds(steerPwm);

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
