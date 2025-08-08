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

// PWM pins
#define PIN_sSteer 0
#define PIN_sShift 2
#define PIN_SEL  4
#define PIN_mSteer 5
#define PIN_mShift 7


class RCXPWM {
public:



void loopCode();

void setMode(String m);
void setFailsafeActive(bool failsafe);
void configPWM();

void setSteerPct(float steerPct);
void setShiftGear(String shiftGear);

float getSteerPct();
String getShiftGear();

void pin_mSteer_interruptX(void);
void pin_mShift_interruptX (void);

private:


// PWM values 
const int steerCenter = 1550; //1575*0.985; // -1.5* correction to balance "toe"
const int steerRightMax = 2000;
const int steerLeftMax = 1000;
const int shiftLow = 1100;
const int shiftHigh = 1900;
const int SEL_RCVR = 1000;
const int SEL_COMP = 2000;
const int pwmPerNominal = 11000;

unsigned long mSteer_micros_last0 = 0, mSteer_micros_last1 = 0;
bool mSteer_meas_rdy = false;
int mSteer_per=0, mSteer_wid=0;

unsigned long mShift_micros_last0 = 0, mShift_micros_last1 = 0;
bool mShift_meas_rdy = false;
int mShift_per=0, mShift_wid=0;

bool shiftState = false;

bool failsafeActive = false;

// Computer (slave mux input) signals
float sSteerPct = 0;
String sShiftGear = "low";

String mode = "bypass"; // default

};


#endif /* ifdef __RCXPWM__*/