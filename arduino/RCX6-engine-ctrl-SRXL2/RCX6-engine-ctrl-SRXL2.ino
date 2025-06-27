/* *************************************

************************************* */

#include <Arduino_JSON.h>
#include <strings.h>

// SRXL2 Throttle Pins
#define RX0 13
#define TX0 12
#define TX0EN 11
#define TX1EN 10
#define RX1 9
#define TX1 8
#define BYPASS 3

void configPins(){
  pinMode(BYPASS, OUTPUT);
  digitalWrite(BYPASS, 0); // start in bypass mode

  pinMode(TX0EN, OUTPUT);
  digitalWrite(TX0EN, 1); // disable UART1 TX pin switch

  pinMode(TX1EN, OUTPUT);
  digitalWrite(TX1EN, 1); // disable UART2 TX pin switch
}

// Spektrum SRXL header
typedef struct SrxlHeader
{
    uint8_t srxlID;     // Always 0xA6 for SRXL2
    uint8_t packetType;
    uint8_t length;
} PACKED SrxlHeader;

// Channel Data
typedef struct SrxlChannelData
{
    int8_t    rssi;         // Best RSSI when sending channel data, or dropout RSSI when sending failsafe data
    uint16_t  frameLosses;  // Total lost frames (or fade count when sent from Remote Rx to main Receiver)
    uint32_t  mask;         // Set bits indicate that channel data with the corresponding index is present
    uint16_t  values[32];   // Channel values, shifted to full 16-bit range (32768 = mid-scale); lowest 2 bits RFU
} PACKED SrxlChannelData;

// Control Data
typedef struct SrxlControlData
{
    uint8_t cmd;
    uint8_t replyID;
    union
    {
        SrxlChannelData channelData;    // Used for Channel Data and Failsafe Channel Data commands
        SrxlVtxData     vtxData;        // Used for VTX commands
        SrxlFwdPgmData  fpData;         // Used to pass forward programming data to an SRXL device
    };
} PACKED SrxlControlData;

typedef struct SrxlControlPacket
{
    SrxlHeader      hdr;
    SrxlControlData payload;
//  uint16_t        crc;    // NOTE: Since this packet is variable-length, we can't use this value anyway
} PACKED SrxlControlPacket;

// Telemetry
typedef struct SrxlTelemetryData
{
    union
    {
        struct
        {
            uint8_t sensorID;
            uint8_t secondaryID;
            uint8_t data[14];
        };
        uint8_t raw[16];
    };
} PACKED SrxlTelemetryData;

typedef struct SrxlTelemetryPacket
{
    SrxlHeader          hdr;
    uint8_t             destDevID;
    SrxlTelemetryData   payload;
    uint16_t            crc;
} PACKED SrxlTelemetryPacket;

// Telemetry messages
#define	TELE_DEVICE_TEXTGEN			(0x0C)										// Text Generator
#define	TELE_DEVICE_ESC			 	  (0x20)										// Electronic Speed Control
#define	TELE_DEVICE_SMARTBATT		(0x42)										// Spektrum SMART Battery

//////////////////////////////////////////////////////////////////////////////
//
//							0X0C - TEXT GENERATOR
//
//////////////////////////////////////////////////////////////////////////////
//
typedef struct
{
	UINT8		identifier;
	UINT8		sID;															// Secondary ID
	UINT8		lineNumber;														// Line number to display (0 = title, 1-8 for general, 254 = Refresh backlight, 255 = Erase all text on screen)
	char		text[13];														// 0-terminated text when < 13 chars
} STRU_TELE_TEXTGEN;

//////////////////////////////////////////////////////////////////////////////
//
//							0X20 - ESC
//
//////////////////////////////////////////////////////////////////////////////
//
//	Uses big-endian byte order
//
typedef struct
{
	UINT8		identifier;														// Source device = 0x20
	UINT8		sID;															// Secondary ID
	UINT16		RPM;															// Electrical RPM, 10RPM (0-655340 RPM)  0xFFFF --> "No data"
	UINT16		voltsInput;														// Volts, 0.01v (0-655.34V)       0xFFFF --> "No data"
	UINT16		tempFET;														// Temperature, 0.1C (0-6553.4C)  0xFFFF --> "No data"
	UINT16		currentMotor;													// Current, 10mA (0-655.34A)      0xFFFF --> "No data"
	UINT16		tempBEC;														// Temperature, 0.1C (0-6553.4C)  0xFFFF --> "No data"
	UINT8		currentBEC;														// BEC Current, 100mA (0-25.4A)   0xFF ----> "No data"
	UINT8		voltsBEC;														// BEC Volts, 0.05V (0-12.70V)    0xFF ----> "No data"
	UINT8		throttle;														// 0.5% (0-100%)                  0xFF ----> "No data"
	UINT8		powerOut;														// Power Output, 0.5% (0-127%)    0xFF ----> "No data"
} STRU_TELE_ESC;


byte packetDataRX0[100];
// returns packet ID type, 0xCD, 0x80, ...
int getPacketDataRX0(){
  SerialUART *ser_p = &Serial1;
  byte *buff = packetDataRX0;
  int buffLen = sizeof(packetDataRX0);
  return getPacketDataRXn(ser_p, buff, buffLen);
}

byte packetDataRX1[100];
// returns packet ID type, 0xCD, 0x80, ...
int getPacketDataRX1(){
  SerialUART *ser_p = &Serial2;
  byte *buff = packetDataRX1;
  int buffLen = sizeof(packetDataRX1);
  return getPacketDataRXn(ser_p, buff, buffLen);
}

// returns packet ID (type) 0xCD, 0x80, ... 
// else -1 if bad packet or 0 if packet not read
int getPacketDataRXn(SerialUART *ser_p, byte *buff, int buffLen) {
  if(ser_p->available()==0) return 0;
  byte d = 0;
  int n = 0;
  int packetType = 0;
  int packetLength = 0;

  // wait for start of frame 0xA6
  n = ser_p->readBytes(&d, 1);
  if(n==1 && d==0xA6) { 
    // SOF detected - get packet data before returning
    buff[0]=d;
    ser_p->readBytes(&buff[1], 2);
    packetType = buff[1];
    packetLength = buff[2];
    if(packetLength>0 && packetLength<buffLen) { 
      int len = packetLength - 3;
      ser_p->readBytes(&buff[3], len);
      uint16_t crc0 = calcCRC(buff, packetLength);
      uint16_t crc1 = ((uint16_t)buff[packetLength-2]<<8 | buff[packetLength-1]);
      if(crc0 != crc1) packetType = -1;
    } else {
      // Discard data in serial buffer
      while(ser_p->available()) ser_p->read();
      packetType=0;
    }
  }
  return packetType;
};

void printPacketRX0(int id){
  printPacket("RX0: ", packetDataRX0, id);
}
void printPacketRX1(int id){
  printPacket("RX1: ", packetDataRX1, id);
}

String mode_g = "bypass";

void printPacket(String s, byte *buff, int id){
  if(id == 0) return;
  Serial.print(mode_g);
  Serial.print(s);
  if(id==-1) {
    Serial.println("BAD CRC");
  } else {
    if(id!=0x0 && id==buff[1]){
      int len = buff[2];
      Serial.print(len); Serial.print(" ");
      for(int i=0;i<len;i++) {
        byte d = buff[i];
        if(d<0x10) Serial.print('0');
        Serial.print(d, HEX); Serial.print(',');
      }
      Serial.println();
    }
  }
}

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
  digitalWrite(TX0EN, 1); // disable UART TX pin switch
  digitalWrite(TX1EN, 1); // disable UART TX pin switch

}


/****************************************************/
void setup() {

  configPins();

  Serial.begin(115200);
  while(!Serial) delay(100);
  Serial.println("USB Serial started");

  Serial1.setRX(RX0);
  Serial1.setTX(TX0);
  Serial1.begin(115200);
  while(!Serial1) delay(100);
  Serial1.setTimeout(2);
  Serial.println("USB Serial1 started");

  Serial2.setRX(RX1);
  Serial2.setTX(TX1);
  Serial2.begin(115200);
  while(!Serial2) delay(100);
  Serial2.setTimeout(2);
  Serial.println("USB Serial2 started");
}

void packetPassthruRX1(int id){
  if(mode_g!="passthru") return;
  if(id!=0xCD) return;
  // pass control data packet from receiver to ESC
  int packetID = packetDataRX1[1];
  if(packetID!=id) return;
  int packetLen = packetDataRX1[2];
//Serial.print("TX0: "); Serial.println(packetLen);
  // Check for RX0 active before transmitting?
  digitalWrite(TX0EN, 0); // enable UART1 TX pin switch
  Serial1.write(packetDataRX1, packetLen);
  delay(1);
  Serial1.flush(); // wait for TX0 transmission complete
  digitalWrite(TX0EN, 1); // disable UART1 TX pin switch
}


void packetPassthruRX0(int id){
  if(mode_g!="passthru") return;
  if(id!=0x21 && id!=0x80) return;
  // pass control data packet from receiver to ESC
  int packetID = packetDataRX0[1];
  if(packetID!=id) return;
  int packetLen = packetDataRX0[2];
//Serial.print("TX1: "); Serial.println(packetLen);

  // Check for RX1 active before transmitting?
  digitalWrite(TX1EN, 0); // enable UART0 TX pin switch
  Serial2.write(packetDataRX0, packetLen);
  Serial2.flush(); // wait for TX1 transmission complete
  digitalWrite(TX1EN, 1); // disable UART0 TX pin switch
}

/****************************************************/
void loop() {
  getJsonMsgs();

  int id = 0;
  id = getPacketDataRX0();
  packetPassthruRX0(id);
  printPacketRX0(id);

  id = getPacketDataRX1();
  packetPassthruRX1(id);
  printPacketRX1(id);

}

uint16_t calcCRC(byte *packet, int length) {
        // Use bitwise method
        uint16_t crc = 0x0000;
        for(uint8_t i = 0; i < length-2; i++)
        {
            crc = crc ^ ((uint16_t)packet[i] << 8);
            for(int b = 0; b < 8; b++)
            {
                if(crc & 0x8000)
                    crc = (crc << 1) ^ 0x1021;
                else
                    crc = crc << 1;
            }
        }
        return crc;
}