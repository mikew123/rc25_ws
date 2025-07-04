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

// Spektrum SRXL header
typedef struct SrxlHeader
{
    uint8_t srxlID;     // Always 0xA6 for SRXL2
    uint8_t packetType;
    uint8_t length;
} PACKED SrxlHeader;


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

// Telemetry
typedef struct SrxlTelemetryData
{
    union
    {
        struct
        {
          uint8_t sensorID;
          uint8_t secondaryID;
        };
        STRU_TELE_ESC esc;
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


union srxlPkt {
  uint8_t b[100];
  SrxlHeader hdr;
  SrxlTelemetryPacket tPacket;
  SrxlControlPacket cPacket;
};


#endif /* ifndef __SRXL2STRUCTS__*/