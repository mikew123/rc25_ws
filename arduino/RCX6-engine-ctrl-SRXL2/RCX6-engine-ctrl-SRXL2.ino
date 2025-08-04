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
PioEncoder shaftEncoder = {PIN_SHAFT_ODOM_A};
#define ODOM_PER_USEC 333 /* 30 Hz odom encoder sample rate, PID */
// OFFSETS to stagger the processing in the interrupt code
#define ODOM_OFFSET_USEC 100
#define PID_OFFSET_USEC 200
#define OMSG_OFFSET_USEC 300

// ODOM POD
#define ODOM_RADUIS_M (0.048/2)
#define ODOM_CIRCUMFERENCE_M (2*PI*ODOM_RADUIS_M)
#define ODOM_ENCODER_COUNT_PER_ROTATION (2000.0)

uint32_t odomTimerLastMs = 0;

int encoderPolarity = 1;
volatile int32_t currEncoderCount[2] = {0, 0}; // [0]: millis timestamp, [1]: encoder count

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

  noInterrupts();
  currEncoderCount[0] = millis(); // timestamp
  currEncoderCount[1] = 0;        // encoder count
  interrupts();

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
uint32_t odomTickCounter = 0;
uint32_t odomTickCount = 0;
uint32_t pidTickCount = 0;
uint32_t omsgTickCount = 0;
bool sendOdom = false;
bool runPID = false;

void OdomHandler() {

  // odomCurrTimeMs = millis();
  // if(odomCurrTimeMs < (odomLastTimeMs + ODOM_PER_MSEC)) return;

  // Read the drive shaft odom encoder
  if(odomTickCounter > odomTickCount) {
    odomDiffTimeMs = odomCurrTimeMs - odomLastTimeMs;
    odomLastTimeMs = odomCurrTimeMs;
    // read pod encoder
    noInterrupts();
    currEncoderCount[0] = millis(); // timestamp
    currEncoderCount[1] = shaftEncoder.getCount() * encoderPolarity; // encoder count
    interrupts();
    // Next odom sample time
    odomTickCount = odomTickCounter + ODOM_PER_USEC;
  }

  // PID code
  if(odomTickCounter > pidTickCount) {
    // protect stamp,count pair from changing by disabling interrupts
    noInterrupts();
    diffEncoderCount = currEncoderCount[1] - lastEncoderCount;
    lastEncoderCount = currEncoderCount[1];
    interrupts();

    runPID = true;

    // Next PID process time
    pidTickCount = odomTickCounter + ODOM_PER_USEC;
  }
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

  // PID code
  if(odomTickCounter > omsgTickCount) {
    sendOdom = true;
    // Next Odom message time
    omsgTickCount = odomTickCounter + ODOM_PER_USEC;
  }

  odomTickCounter++;
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
float sSteerPct = 0.0; 
String sShiftGear = "low";

// Loop timer for precise interval periods
unsigned long statusMillis_last = 0;

String mode_g = "bypass"; // power/reset default
int loopStatusPeriod = 1000;

/***************************************
* Hardware PICO TIMER 
*******************************************/
#define TX_ESC_USEC 33333 // 30/sec
// Configure the timer for the SRXL2, used to time TXEN signals
// Also used for Odom wheel encoder
void configTimer() {
  srx.setTimerTickIntervalUsec(TIMER_PER_USEC);
  srx.setTxEscUsec(TX_ESC_USEC); // also syncronizes

  // syncronize the Odom timer
  odomTickCount = odomTickCounter + ODOM_PER_USEC + ODOM_OFFSET_USEC;
  // syncronize the PID loop timer
  pidTickCount = odomTickCounter + ODOM_PER_USEC + PID_OFFSET_USEC;
  // syncronize the odom message timer
  omsgTickCount = odomTickCounter + ODOM_PER_USEC + OMSG_OFFSET_USEC;

  if (ITimer.attachInterruptInterval(TIMER_PER_USEC, timerTick))
    {Serial.print(F("Starting ITimer OK, period usec = ")); Serial.println(TIMER_PER_USEC);}
  else
    {Serial.println(F("Can't set ITimer. Select another freq. or timer"));}
  Serial.flush();  
}

// Tick the timer in SRXL2 and execute Odom Wheel encoder
// Interrupt drive at 10 KHz
bool timerTick(struct repeating_timer *t) { 
  (void) t;
  srx.timerTick();
  OdomHandler();
  watchdogTimer();
  return true;
}

/********************************************************
* Watchdog timer
* Called from timer "Tick" interrupt
*********************************************************/
uint32_t wdTimeStopMsec = 0; // watchdog time in msec, 0=disabled
uint32_t wdTimeMsec = 0;
bool wdTimedOut = true;
// Called from interrupt 
void watchdogTimer() {
  if (wdTimeStopMsec > 0) {
    if (millis() > wdTimeStopMsec) {
      // watchdog timeout
      wdTimedOut = true;
      wdTimeStopMsec = 0; // disable watchdog until new command
      Serial.println("Watchdog timeout, stopping motor");
      sThrottlePct = 0;
      sSteerPct = 0;
      resetPID();
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
  wdTimedOut = false;
}

/********************************************************
* PID loop code 
* Uses linear M/S from command with the shaft encoder feedback
* And executes the Ackermann conversion after each PID sample
* The angular velocity does not have a PID loop (no servo encoder)
*********************************************************/
float coeffA = 0.5;  // proportional scale
float coeffB = 0.05; // integral scale
float coeffDA = 0.0;  // diff proportional scale
float coeffDB = 0.0; // diff integral scale
float coeffDd = 0.5; // diff decay rate
int encPerMeter = 6000; // encoder counts per meter

float pidInt = 0;
float pidDd = 0; // Diff signal decay
float pidMaxMps = 2.0; // +- maximum meters/sec output and integrator

// TODO: timestamps are uint32_t?
int32_t pidLastStamp = 0;
int32_t pidLastEnc = 0;

// data for the median-3 filter
float medianData[3] = {0,0,0};

// from "cv":[linX,angZ] command velocity from serial port
float cmdVelLinX = 0;
float cmdVelAngZ = 0;

// Add global variables for last commanded velocities
float lastLinX = 0.0;
float lastAngZ = 0.0;
float lastCmdVelLinX = 0;

void resetPID(){
  runPID = false;
  cmdVelLinX = 0;
  cmdVelAngZ = 0;
  lastLinX = 0;
  lastAngZ = 0;
  lastCmdVelLinX = 0;
  sThrottlePct = 0;
  sSteerPct = 0;
  pidInt = 0;
  pidDd = 0;
  pidLastEnc = 0;
  for(int i=0;i<3;i++) medianData[i]=0;

}

void pidLoopCode(){
  if(runPID!=true) return;
  runPID = false;
  if(mode_g!="cv" || wdTimedOut==true) return;

  // PID code
  float linX = 0;
  float angZ = 0;
  float encMps = 0;
  float err = 0;
  float propErr = 0;
  float intErr = 0;

  int32_t stamp = 0;
  int32_t enc = 0;
  float encTsec = 0;
  float deltaEnc = 0;
  
  float sortM3[3];

  // protect timestamp and encoder value pair from interrupt
  noInterrupts();
  stamp = currEncoderCount[0]; // timestamp
  enc   = currEncoderCount[1]; // encoder count
  interrupts();


  /****************** start PID *********************/

  // encTsec = (stamp - pidLastStamp)/1000.0; // delta stamp time in msec to seconds
  // pidLastStamp = stamp;
  // if(encTsec>0.1 || encTsec<=0) return; // bad sample
  encTsec = 1e-6*ODOM_PER_USEC*TIMER_PER_USEC; // reduce jitter in time

  deltaEnc = enc - pidLastEnc;
  pidLastEnc = enc;
  if(deltaEnc > 15000 || deltaEnc < -15000) return; // bad sample

  // filter deltaEnc uing a median filter - reduce jitter in encoder data
  medianData[1]=medianData[0]; medianData[2]=medianData[1]; // shift in new data
  medianData[0]=deltaEnc;
  for(int i=0;i<3;i++) sortM3[i]=medianData[i];
  if(sortM3[1]<sortM3[0]) std::swap(sortM3[1],sortM3[0]);
  if(sortM3[2]<sortM3[0]) std::swap(sortM3[2],sortM3[0]);
  if(sortM3[2]<sortM3[1]) std::swap(sortM3[2],sortM3[1]);
  float filtEnc = sortM3[1];

  encMps = (filtEnc/encPerMeter)/encTsec; // convert encoder counts to meters/sec

  err = cmdVelLinX - encMps; 
  propErr = err*coeffA;
  intErr  = err*coeffB;

  pidInt += intErr;

  // Add differential signal from chance of input
  float diff = cmdVelLinX - lastCmdVelLinX;
  lastCmdVelLinX = cmdVelLinX;
  pidDd += diff - pidDd*coeffDd;
  float propDerr = pidDd*coeffDA;
  float intDerr = pidDd*coeffDB;
  if(fabs(intDerr) < 2.0) pidInt += intDerr;

  // Limit integrator value
  if(pidInt>pidMaxMps) pidInt = pidMaxMps;
  else if(pidInt<-pidMaxMps) pidInt = -pidMaxMps;

  linX = propErr + pidInt + propDerr;
  
  // Limit PID output value
  if(linX>pidMaxMps) linX = pidMaxMps;
  else if(linX<-pidMaxMps) linX = -pidMaxMps;

  /****************** end PID *********************/
Serial.print("PID");
Serial.print(", stamp=");
Serial.print(stamp);
Serial.print(", encTsec=");
Serial.print(encTsec, 4);
Serial.print(", filtEnc=");
Serial.print(filtEnc);
Serial.print(", encMps=");
Serial.print(encMps, 4);
Serial.print(", err=");
Serial.print(err, 4);
Serial.print(", propErr=");
Serial.print(propErr, 4);
Serial.print(", intErr=");
Serial.print(intErr, 4);
Serial.print(", pidInt=");
Serial.print(pidInt, 6);
Serial.print(", linX=");
Serial.print(linX, 4);
Serial.println();

  // Ackerman conversion and output to ESC via RXCL2
  //TODO: move output to this function?
  AckermanConvert(linX, cmdVelAngZ);
}

/********************************************************
* Ackerman conversion
* Convert linear and angular velocity to steering and throttle percent
*********************************************************/
const float wheelCircumference = 750.0; // mm
const float odomCountPerWheelRotation = 3201.6;

// conversion  of linear x to throttle percent, + and - are different
// const float throttleFwdPctPerMps = 53.18*22/25; // 47.04*(1/0.87)*(3/3.05);
// const float throttleRvsPctPerMps = 53.0*22/25; // was not tweaked
const float throttlePctPerMps = 46.80; // use for both fwd and rvs
const float throttlePctDeadZone = 10.0; // this is +- dead zone around 0 percent

// M/S to Percent vs Vbat correction scale factor equation = A*Vbat + B
const float throttlePctVbatScaleA = 0.078545; // from test deltaEnc vs Vbat on bench 
const float throttlePctVbatScaleB = 0.018182; // from test deltaEnc vs Vbat on bench

// Ackermann steering parameters
const float wheelBase = 0.490; // meters, front to back
const float trackWidth = 0.310; // meters, left to right
// const float maxSteeringRad = 0.523; // ~30 deg, adjust for your hardware
const float maxSteeringRad = 0.611; // ~35 deg at steer percent=100

float throttlePctPerMpsScale = 1.0;  // mpy 1.0 = no scale
float throttlePctDeadZoneAdj = 0.0;  // add Adj
float steerCenterPctAdj = 0.0;       // add Adj
float wheelBaseAdj = 0.0;            // add Adj

// TODO: read values in PID loop and calculate using scale in PID loop?
// currently these are read in the command process loop
float escVin = 12.0; // default value
int motorRpm = 0; // default value

// Ackerman conversion called when serial command is decoded
void AckermanConvert(float linX, float angZ) {
  // Ackerman steering conversion
  // linear X is in m/s, angular Z is in rad/s  

  // Throttle calculation
  // TODO: adjust for high and low gear, current is for low gear only
  float scale = (throttlePctVbatScaleA * escVin) + throttlePctVbatScaleB;
  float dz = throttlePctDeadZone + throttlePctDeadZoneAdj; // adjust dead zone
  float ppm = throttlePctPerMps * throttlePctPerMpsScale; // scale percent to meters
  if(linX==0.0) sThrottlePct = 0;
  else if(linX>0.0) {
    sThrottlePct = (dz-1) + (linX * ppm);
    sThrottlePct /= scale; // apply Vbat correction
    if(sThrottlePct > 100) sThrottlePct = 100;
    if(sThrottlePct < dz) sThrottlePct = 0;
    // // convert back to linear velocity x m/s
    // linX = (sThrottlePct - (dz-1)) / ppm;
    // linX *= scale; // apply reverse Vbat correction
  } else {
    sThrottlePct = -(dz-1) + (linX * ppm);
    sThrottlePct /= scale; // apply Vbat correction
    if(sThrottlePct < -100) sThrottlePct = -100;
    if(sThrottlePct > -dz) sThrottlePct = 0;
    // // convert back to linear velocity x m/s
    // linX = (sThrottlePct + (dz-1)) / ppm;
    // linX *= scale; // apply reverse Vbat correction
  }

  // Serial.print("throttle pct = ");
  // Serial.print(sThrottlePct);
  // Serial.print(" M/s = ");
  // Serial.println(linX);

  // Ackermann steering calculation
  float wb = wheelBase + wheelBaseAdj; // Adjust the wheel base length
  float steeringAngleRad = atan2(wb * angZ, linX);
  // manage negative linear velocity using atan2
  if (linX < 0) {
    if(angZ>=0) steeringAngleRad -= 3.14159;
    else steeringAngleRad += 3.14159;
  }

  // Convert steering angle to percent
  sSteerPct = (100.0 * (steeringAngleRad / maxSteeringRad)) + steerCenterPctAdj;
  if(sSteerPct > 100) sSteerPct = 100;
  if(sSteerPct < -100) sSteerPct = -100;
  // convert back to angular z radians/sec
  angZ = (sSteerPct / 100.0) * maxSteeringRad * linX / wheelBase;

  // Reverse conversion
  steeringAngleRad = ((sSteerPct - steerCenterPctAdj) / 100.0) * maxSteeringRad;
  angZ = tan(steeringAngleRad) * linX / wheelBase;

  // Serial.print("steer");
  // Serial.print(" angZ = ");
  // Serial.print(angZ);
  // Serial.print(" linX = ");
  // Serial.print(linX);
  // Serial.print(" Rad = ");
  // Serial.print(steeringAngleRad);
  // Serial.print(" pct = ");
  // Serial.print(sSteerPct);
  // Serial.print(" Rad/s = ");
  // Serial.println(angZ);

  srx.setThrottlePct(sThrottlePct);
  pwm.setSteerPct(sSteerPct);

  // Save last velocities to ESC
  lastLinX = linX;
  lastAngZ = angZ;
}


/********************************************************
* USB serial JSON code
*********************************************************/
// retrieve JSON messages from USB serial port
void getJsonMsgs() {
  if (Serial.available()== 0) return;

  // read the incoming string and parse it
  String incomingString = Serial.readStringUntil('\n');
  jsonParse(incomingString.c_str());

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

  // mode bypass/passthru/cv/pct
  // bypass and passthru use the RC transmitter to control
  // cv and pct use the serial commands:
  //  cv for command velocity style "cv":[linX, angZ] radians/sec float
  //  pct for percent control "thr":N and "str":N percent float
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

  if(mode_g == "cv") {
    // Command Velocity array [linear_X, angular_Z]
    if (myObject.hasOwnProperty("cv")) {
      JSONVar cv;
      cv = myObject["cv"];
      Serial.print("CmdVar = [lx=");
      Serial.print((double)cv[0]);
      Serial.print(", az=");
      Serial.print((double)cv[1]);
      Serial.println("]");

      cmdVelLinX = (double)cv[0];
      cmdVelAngZ = (double)cv[1];
    }

    // PID loop coefficients [prop, int, diffProp, diffInt]
    if (myObject.hasOwnProperty("pid")) {
      JSONVar pid;
      pid = myObject["pid"];
      coeffA = (double)pid[0];
      coeffB = (double)pid[1];
      coeffDA = (double)pid[2];
      coeffDB = (double)pid[3];

      Serial.print("pid = [coeffA=");
      Serial.print(coeffA);
      Serial.print(", coeffB=");
      Serial.print(coeffB);
      Serial.print(", [coeffDA=");
      Serial.print(coeffDA);
      Serial.print(", coeffDB=");
      Serial.print(coeffDB);
      Serial.println("]");
    }
  } else {
    cmdVelLinX = 0;
    cmdVelAngZ = 0;
  }


  if (mode_g == "pct") {
    // Steering percent
    if (myObject.hasOwnProperty("str")) {
      sSteerPct = (double) myObject["str"];
      Serial.print("steer = ");
      Serial.println(sSteerPct);
      // add steering percent offset
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
  } 
  else {
    sThrottlePct = 0;
    sSteerPct = 0;
  }


  // Shift gear high/low
  if (myObject.hasOwnProperty("sft")) {
    sShiftGear = (String) myObject["sft"];
    Serial.print("gear = ");
    Serial.println(sShiftGear);
    pwm.setShiftGear(sShiftGear=="high"?"high":"low");
    srx.setShiftGear(sShiftGear=="high"?"high":"low");
  }

  // Throttle trim - floats - 0 is no trim for all
  // [
  // throttlePctPerMpsScale,  mpy scale 
  // throttlePctDeadZoneAdj,  add Adj
  // steerCenterPctAdj,       add Adj
  // wheelBaseAdj             add Adj
  //]
  if (myObject.hasOwnProperty("trim")) {
    JSONVar trim;
    trim = myObject["trim"];
    throttlePctPerMpsScale = 1+(double)trim[0]; // trim[0]==0 no trim
    throttlePctDeadZoneAdj = (double)trim[1];
    steerCenterPctAdj      = (double)trim[2];
    wheelBaseAdj           = (double)trim[3];
    Serial.print("throttlePctPerMpsScale = ");
    Serial.print(throttlePctPerMpsScale, 3);
    Serial.print(", throttlePctDeadZoneAdj = ");
    Serial.print(throttlePctDeadZoneAdj, 3);
    Serial.print(", steerCenterPctAdj = ");
    Serial.print(steerCenterPctAdj, 3);
    Serial.print(", wheelBaseAdj = ");
    Serial.println(wheelBaseAdj, 3);
  }


  // get Battery Voltage and RPM from telemetry
  // move to PID loop?
  escVin = srx.getEscVin();
  motorRpm = srx.getEscRpm();

  return true;
}

//TODO: move to SRXL2
void configureMode(){
  if(mode_g == "bypass") {
    digitalWrite(BYPASS, 0); // enable bypass switch 
  }
  else if(mode_g == "passthru" || mode_g == "cv" || mode_g == "pct") {
    digitalWrite(BYPASS, 1); // disable bypass switch
  }

  // to be safe reset PID when mode changes
  resetPID();

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

void sendOdomMsg() {
  JSONVar myObject;
  uint32_t stamp, enc;

  noInterrupts();
  stamp = currEncoderCount[0]; // timestamp
  enc = currEncoderCount[1]; // encoder count
  interrupts();

  myObject["stamp"] = (uint32_t)stamp;
  myObject["enc"] = (int32_t)enc; // encoder values are + or -

  myObject["linx"] = (float)lastLinX;
  myObject["int"] = (float)pidInt;

  String jsonString = JSON.stringify(myObject);
  Serial.println(jsonString);
}

// Send status message to host computer
void sendStatusMsg() {

  JSONVar myObject;

  myObject["mode"] = mode_g;

  myObject["gear"] = pwm.getShiftGear();

  // myObject["fsa"] = failsafeActive;

  // myObject["rca"] = receiverSignalsValid;

  // add ESV Vin and RPM
  myObject["vbat"] = escVin;

  myObject["rpm"] = motorRpm; 

  // Add last commanded linear X and limited angular Z
  myObject["linx"] = lastLinX;
  myObject["angz"] = lastAngZ;

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

// Send messages to serial port (to ROS node)
// Called from loop code
void sendMsgs(unsigned long loopMillis) {
  // send status message 1/sec (every loopStatusPeriod msec)
  if ((loopMillis - statusMillis_last) >= loopStatusPeriod) {
    sendStatusMsg();
    statusMillis_last = loopMillis;
  }

  // send Odom wheel encoder data
  if (sendOdom) {
    sendOdom = false;
    sendOdomMsg();
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

  pidLoopCode();

  sendMsgs(loopMillis);

}
