/* *************************************

************************************* */

#include <strings.h>
#include <Arduino_JSON.h>

#include "RPi_Pico_TimerInterrupt.h"
RPI_PICO_Timer ITimer(0);

// local files in arduino sketch folder
#include "srxl2.h"
SRXL2 srx;



#define TIMER_PER_USEC 100



String mode_g = "bypass";



// Configure the timer for the SRXL2, used to time TXEN signals
//bool pin15 = 0;
void configTimer() {
  srx.configTimerTickIntervalUsec(TIMER_PER_USEC);
  if (ITimer.attachInterruptInterval(TIMER_PER_USEC, timerTick))
    {Serial.print(F("Starting ITimer OK, period usec = ")); Serial.println(TIMER_PER_USEC);}
  else
    {Serial.println(F("Can't set ITimer. Select another freq. or timer"));}
  Serial.flush();  

//  pinMode(15, OUTPUT);
//  digitalWrite(15, pin15);

}
// Tick the timer in SRXL2 
bool timerTick(struct repeating_timer *t) { 
  (void) t;
  srx.timerTick();
//  pin15 = !pin15;
//  digitalWrite(15, pin15);

  return true;
}


/*********************************************************/
// USB serial JSON code
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
  }

  // if (myObject.hasOwnProperty("str")) {
  //   sSteerPct = (int) myObject["str"];
  //   Serial.print("steer = ");
  //   Serial.println(sSteerPct);
  // }

  // if (myObject.hasOwnProperty("thr")) {
  //   sThrottlePct = (int) myObject["thr"];
  //   Serial.print("throttle = ");
  //   Serial.println(sThrottlePct);
  // }

  // if (myObject.hasOwnProperty("sft")) {
  //   sShiftGear = (String) myObject["sft"];
  //   Serial.print("gear = ");
  //   Serial.println(sShiftGear);
  // }

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
  digitalWrite(TX0EN, 1); // disable UART1 TX0EN
  digitalWrite(TX1EN, 1); // disable UART2 TX1EN
}

void configSerial(){
  Serial.begin(115200);
  while(!Serial) delay(100);
  Serial.println("USB Serial started");
}

/****************************************************/
void setup() {
  srx.configPins();
  configSerial();
  srx.configSerialESC();
  srx.configSerialRCV();
  configTimer();
}

/****************************************************/
void loop() {
  getJsonMsgs();

  int id = 0;
  id = srx.getPacketDataRX0();
  srx.packetPassthruRX0(id);
  srx.sendPacketTX1();

  // srx.printPacketRX0(id);

  id = srx.getPacketDataRX1();
  srx.packetPassthruRX1(id);
  srx.sendPacketTX0();
  // srx.printPacketRX1(id);

}
