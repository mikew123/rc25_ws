#ifndef __SRXL2STRUCTS__
#define __SRXL2STRUCTS__



// Enable byte packing for all structs defined here!
#ifdef PACKED
#error "preprocessor definition PACKED is already defined -- this could be bad"
#endif

#ifdef __GNUC__
#define PACKED __attribute__((packed))
#else
#pragma pack(push, 1)
#define PACKED
#endif

// SRXL2 Packet header structures
// Reverse bytes in 16 bit data from ESC (not from receiver!)
#define RVRSB(W) (((W&0xFF)<<8) | (W>>8))

// SRXL2 packetsm****************************************************************
// Spektrum SRXL header for all packet types
typedef struct SrxlHeader
{
    uint8_t srxlID;     // Always 0xA6 for SRXL2
    uint8_t packetType;
    uint8_t length;
} PACKED SrxlHeader;


// SRXL2 ESC channel data 
typedef struct SrxlEscChannelUsed
{
  uint16_t throttle;
  uint16_t steer;
  uint16_t shift;
} SrxlEscChannelUsed;

// Channel Data
typedef struct SrxlChannelData
{
    int8_t    rssi;         // Best RSSI when sending channel data, or dropout RSSI when sending failsafe data
    uint16_t  frameLosses;  // Total lost frames (or fade count when sent from Remote Rx to main Receiver)
    uint32_t  mask;         // Set bits indicate that channel data with the corresponding index is present
    union {
      uint16_t  values[32];   // Channel values, shifted to full 16-bit range (32768 = mid-scale); lowest 2 bits RFU
      SrxlEscChannelUsed esc; // Channels used in RCX6 packet from receiver to ESC
    };
} PACKED SrxlChannelData;

// Control Data
typedef struct SrxlControlData
{
    uint8_t cmd;
    uint8_t replyID;
    union
    {
        SrxlChannelData channelData;    // Used for Channel Data and Failsafe Channel Data commands
    };
} PACKED SrxlControlData;

typedef struct SrxlControlPacket
{
    SrxlHeader      hdr;
    SrxlControlData payload;
//  uint16_t        crc;    // NOTE: Since this packet is variable-length, we can't use this value anyway
} PACKED SrxlControlPacket;


// ************************************************************************
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
	uint8_t		identifier;
	uint8_t		sID;															// Secondary ID
	uint8_t		lineNumber;														// Line number to display (0 = title, 1-8 for general, 254 = Refresh backlight, 255 = Erase all text on screen)
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
  // NOTE: 16 bit values need the bytes reversed to be uint16_t data type compatible
	uint8_t		identifier;		// Source device = 0x20
	uint8_t		sID;				  // Secondary ID
	uint16_t	rpm;					// Electrical RPM, 10RPM (0-655340 RPM)  0xFFFF --> "No data"
	uint16_t	voltsInput;		// Volts, 0.01v (0-655.34V)       0xFFFF --> "No data"
	uint16_t	tempFET;			// Temperature, 0.1C (0-6553.4C)  0xFFFF --> "No data"
	uint16_t	currentMotor;	// Current, 10mA (0-655.34A)      0xFFFF --> "No data"
	uint16_t	tempBEC;			// Temperature, 0.1C (0-6553.4C)  0xFFFF --> "No data"
	uint8_t		currentBEC;		// BEC Current, 100mA (0-25.4A)   0xFF ----> "No data"
	uint8_t		voltsBEC;			// BEC Volts, 0.05V (0-12.70V)    0xFF ----> "No data"
	uint8_t		throttle;			// 0.5% (0-100%)                  0xFF ----> "No data"
	uint8_t		powerOut;			// Power Output, 0.5% (0-127%)    0xFF ----> "No data"
} STRU_TELE_ESC;



//////////////////////////////////////////////////////////////////////////////
//
// Smart Battery
//
//////////////////////////////////////////////////////////////////////////////
//
// Uses little-endian byte order for all multi-byte fields
//
typedef struct
{
uint8_t identifier; // Source device = 0x42
uint8_t sID; // Secondary ID
uint8_t typeChannel; // Upper nybble = Message type; Lower nybble = Battery number (0 or 1)
uint8_t msgData[13]; // Message-specific data, determined by upper nybble of typeChannel (see defs below)
} STRU_SMARTBATT_HEADER;

#define SMARTBATT_MSG_TYPE_MASK_BATTNUMBER (0x0F)
#define SMARTBATT_MSG_TYPE_MASK_MSGTYPE (0xF0)

#define SMARTBATT_MSG_TYPE_REALTIME (0x00)
#define SMARTBATT_MSG_TYPE_CELLS_1_6 (0x10)
#define SMARTBATT_MSG_TYPE_CELLS_7_12 (0x20)
#define SMARTBATT_MSG_TYPE_CELLS_13_18 (0x30)
#define SMARTBATT_MSG_TYPE_ID (0x80)
#define SMARTBATT_MSG_TYPE_LIMITS (0x90)

//.................................................. .........................
// Real-time battery data when current sense is available
typedef struct
{
uint8_t identifier; // Source device = 0x42
uint8_t sID; // Secondary ID
uint8_t typeChannel; // Msg type = SMARTBATT_MSG_TYPE_REALTIME | Battery number (0 or 1)
int8_t temperature_C; // Temperature in degrees C, 1 degree increments (-128 = unavailable)
uint32_t dischargeCurrent_mA; // Amount of current being drawn from battery, in mA steps (0xFFFFFFFF = unavailable)
uint16_t batteryCapacityUsage_mAh; // Approximate battery capacity usage, in mAh (0xFFFF = unavailable)
uint16_t minCellVoltage_mV; // Minimum cell voltage of pack, in mV
uint16_t maxCellVoltage_mV; // Maximum cell voltage of pack, in mV
uint8_t rfu[2];
} STRU_SMARTBATT_REALTIME;

//.................................................. .........................
// Real-time cell voltage
typedef struct
{
uint8_t identifier; // Source device = 0x42
uint8_t sID; // Secondary ID
uint8_t typeChannel; // Msg type = SMARTBATT_MSG_TYPE_CELLS_X_Y | Battery number (0 or 1)
int8_t temperature_C; // Temperature in degrees C, 1 degree increments (-128 = unavailable)
uint16_t cellVoltage_mV[6]; // Cell voltage of first 6 cells, in mV (0xFFFF = unavailable)
} STRU_SMARTBATT_CELLS;

//.................................................. .........................
// Smart Battery ID and general info
typedef struct
{
uint8_t identifier; // Source device = 0x42
uint8_t sID; // Secondary ID
uint8_t typeChannel; // Msg type = SMARTBATT_MSG_TYPE_ID | Battery number (0 or 1)
uint8_t chemistry; // 0:LiHv, 1:LiPo, 2:LiIon, 3:LiFe, 4:Pb, 5:Ni-MH/Cd
uint8_t numOfCells; // Number of cells in the battery
uint8_t manufacturer; // 0:BattGo
uint16_t cycles; // Number of charge/discharge cycles recorded (0 = unavailable)
uint8_t uniqueID[8]; // Unique battery ID, manufacturer-specific
// 0: [0] = lower (first) byte of "Customer ID"
// [1-3] = lower 3 bytes of "Special Mark of Battery"
// [4-7] = 4-byte "Manufacturing Date"
} STRU_SMARTBATT_ID;

//.................................................. .........................
// Smart Battery Limits
typedef struct
{
uint8_t identifier; // Source device = 0x42
uint8_t sID; // Secondary ID
uint8_t typeChannel; // Msg type = SMARTBATT_MSG_TYPE_LIMITS | Battery number (0 or 1)
uint8_t rfu;
uint16_t fullCapacity_mAh; // Fully charged battery capacity, in mAh
uint16_t dischargeCurrentRating; // Rated discharge current, in 0.1C
uint16_t overDischarge_mV; // Limit below which battery is likely damaged, in mV
uint16_t zeroCapacity_mV; // Voltage at which LVC protection should activate, in mV
uint16_t fullyCharged_mV; // Voltage reading expected when fully charged, in mV
int8_t minWorkingTemp; // Minimum working temperature in degrees C, 1 degree steps
int8_t maxWorkingTemp; // Maximum working temperature in degrees C, 1 degree steps
} STRU_SMARTBATT_LIMITS; 


// Telemetry packets payload data
typedef struct SrxlTelemetryData
{
    union {
        struct {
          // Overlay of payload header data
          uint8_t sensorID;
          uint8_t secondaryID;
          uint8_t typeChannel; // For smart battery sensorID=0x42
        };
        STRU_TELE_ESC esc;           // sensorID=0x20 
        STRU_TELE_TEXTGEN txt;       // sensorID=0x0C
        STRU_SMARTBATT_REALTIME srt; // sensorID=0x42, type=0x00
        STRU_SMARTBATT_CELLS scl;    // sensorID=0x42, type=0x10
        STRU_SMARTBATT_ID sid;       // sensorID=0x42, type=0x80
        STRU_SMARTBATT_LIMITS slm;   // sensorID=0x42, type=0x90
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


union srxlPkt {
  uint8_t b[100];
  SrxlHeader hdr;
  SrxlTelemetryPacket tPacket;
  SrxlControlPacket cPacket;
};


#endif /* ifndef __SRXL2STRUCTS__*/