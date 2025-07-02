/*****************************************************************************************
rcxpwm.h

Header and class description for PWM pins on the RCX6 engine controller

The PWM outputs use the 107-Arduino-Servo-RP2040 library using PIO
PWM mux selection PWM output muxsel on PIN_SEL
Steering PWM output on PIN_sSteer
Shift PWM output on PIN_sShift

The PWM inputs use pin interrupts and the millis() timer
Steering PWM input on PIN_mSteer
Shift PWM input on PIN_mShift

Mike Williamson 7/2/2025
*****************************************************************************************/

#ifndef __RCXPWM__
#define __RCXPWM__

#include <Arduino.h>

#include <107-Arduino-Servo-RP2040.h>
// Create servo pwm transmitters
static _107_::Servo sSteer, sShift, muxSel;

#define pwmPerNominal 11000

#define PIN_sSteer 0
#define PIN_sShift 2

#define PIN_SEL  4

#define SEL_RCVR 1000
#define SEL_COMP 2000

#define PIN_mSteer 5
#define PIN_mShift 7

// PWM values 
#define steerCenter 1575
#define steerRightMax 2000
#define steerLeftMax 1000

#define shiftLow 1100 // shift is a static value
#define shiftHigh 1900

#define loopStatusPeriod 1000

class RCXPWM {
public:

unsigned long mSteer_micros_last0 = 0, mSteer_micros_last1 = 0;
bool mSteer_meas_rdy = 0;
int mSteer_per=0, mSteer_wid=0;

unsigned long mShift_micros_last0 = 0, mShift_micros_last1 = 0;
bool mShift_meas_rdy = 0;
int mShift_per=0, mShift_wid=0;

bool shiftState = 0;
bool muxSelRcvr = true;


// Computer (slave mux input) signals
int sSteerPct = 0;
String sShiftGear = "low";

String mode;

void crudeTestCode();
void setMode(String m);
void configPWM();
void computerSignals();


//private:

// static void pin_mSteer_interrupt (void);
// static void pin_mShift_interrupt (void);

void pin_mSteer_interruptX(void);
void pin_mShift_interruptX (void);

};


#endif /* ifdef __RCXPWM__*/