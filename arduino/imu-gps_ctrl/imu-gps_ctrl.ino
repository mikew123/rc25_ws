#include <Arduino.h>
#include <Arduino_JSON.h>

#include <Adafruit_BNO08x.h>

#include <SparkFun_u-blox_GNSS_v3.h> //http://librarymanager/All#SparkFun_u-blox_GNSS_v3

#include <QMC5883LCompass.h>

// No reset pin for UART
#define BNO08X_RESET -1

// UART0/Serial1 pins
#define TX0 0
#define RX0 1

// UART1/Serial2 pins
#define TX1 4
#define RX1 5

// Wire I2C0 pins
#define SDA0 8
#define SCL0 9

Adafruit_BNO08x  bno08x(BNO08X_RESET);

SFE_UBLOX_GNSS_SERIAL myGNSS;

QMC5883LCompass compass;

sh2_SensorValue_t sensorValue;

// Global variables

bool g_imuEna = false;
bool g_gpsEna = false;
bool g_cmpEna = false;

int g_gpsInterval = 100;
dynModel g_dyn_model = DYN_MODEL_PORTABLE;
//dynModel g_dyn_model = DYN_MODEL_PEDESTRIAN;
//dynModel g_dyn_model = DYN_MODEL_AUTOMOTIVE;

void setReports(void) {
  uint32_t period = 500000; // 2 Hz
  Serial.println("Setting desired reports");
  if (! bno08x.enableReport(SH2_GYROSCOPE_CALIBRATED, period)) { //0x02
    Serial.print("Could not enable report SH2_GYROSCOPE_CALIBRATED 0x");
    Serial.println(SH2_GYROSCOPE_CALIBRATED,HEX);
  }
  if (! bno08x.enableReport(SH2_LINEAR_ACCELERATION, period)) { //0x04
    Serial.print("Could not enable report SH2_LINEAR_ACCELERATION 0x");
    Serial.println(SH2_LINEAR_ACCELERATION,HEX);
  }
  if (! bno08x.enableReport(SH2_ARVR_STABILIZED_RV, period)) { //0x28
    Serial.print("Could not enable report SH2_ARVR_STABILIZED_RV 0x");
    Serial.println(SH2_ARVR_STABILIZED_RV,HEX);
  }
}

void setupImu(void) {
  Serial1.setTX(TX0);
  Serial1.setRX(RX0);
  Serial1.setFIFOSize(1024);

  if (!bno08x.begin_UART(&Serial1)) {  // Requires a device with > 300 byte UART buffer!
    Serial.println("Failed to find BNO08x chip");
    while (1) { delay(10); }
  }
  Serial.println("BNO08x Found!");

  // Enable sensors 
  setReports();
}

void setupGps(void) {
  Serial2.setTX(TX1);
  Serial2.setRX(RX1);
  
  //Assume that the U-Blox GNSS is running at 9600 baud (the default) or at 115200 baud.
  //Loop until we're in sync and then ensure it's at 115200 baud.
  do {
    Serial2.println("GNSS: trying 115200 baud");
    Serial2.begin(115200);
    if (myGNSS.begin(Serial2) == true) break;

    delay(100);
    Serial.println("GNSS: trying 9600 baud");
    Serial2.begin(9600);
    if (myGNSS.begin(Serial2) == true) {
        Serial.println("GNSS: connected at 9600 baud, switching to 115200");
        myGNSS.setSerialRate(115200);
        delay(100);
        myGNSS.saveConfiguration(); //Save the current settings to flash and BBR
    } else {
        myGNSS.factoryDefault();
        delay(2000); //Wait a bit before trying again to limit the Serial output
    }
  } while(1);
  Serial.println("GNSS serial connected");

  myGNSS.setUART1Output(COM_TYPE_UBX); //Set the UART port to output UBX only
  myGNSS.setMeasurementRate(g_gpsInterval);

  myGNSS.setDynamicModel(g_dyn_model);

}

void setupCmp(void) {
  Wire.setSDA(SDA0);
  Wire.setSCL(SCL0);
   
  compass.init();

// #1
//  compass.setCalibrationOffsets(-121.00, -114.00, 610.00);
//  compass.setCalibrationScales(2.33, 2.47, 0.46);
// #2
//  compass.setCalibrationOffsets(696.00, 443.00, -364.00);
//  compass.setCalibrationScales(0.94, 0.95, 1.13);
// #3
//  compass.setCalibrationOffsets(258.00, 134.00, -396.00);
//  compass.setCalibrationScales(0.92, 0.91, 1.22);
// #4
  compass.setCalibrationOffsets(39.00, 333.00, 830.00);
  compass.setCalibrationScales(1.00, 0.82, 1.27);
// #5
//  compass.setCalibrationOffsets(405.00, 129.00, 42.00);
//  compass.setCalibrationScales(1.05, 0.98, 0.97);
// #6
//  compass.setCalibrationOffsets(505.00, 259.00, 169.00);
//  compass.setCalibrationScales(1.15, 0.91, 0.97);  
// #7
//  compass.setCalibrationOffsets(1017.00, 155.00, 261.00);
//  compass.setCalibrationScales(1.34, 0.85, 0.93);

  compass.setMagneticDeclination(13, 0); // tweaked from Dallas = 2, 37
  compass.setSmoothing(10,true); 

  Serial.println("Compass I2C connected and calibrated");
}

void setup(void) {

  Serial.begin(115200);
  while (!Serial) delay(10);     //  pause  until serial console opens

  Serial.println("IMU + GPS");
  
  setupImu();

  setupGps();

  setupCmp();
  
  delay(10);
}


void loop() {
  serialRx();
  procImu();
  procGps();
  procCmp();
}


void serialRx() {
  // Check serial port for a JSON message
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

  if (myObject.hasOwnProperty("cfg")) {
    return jsonParseCfg(myObject["cfg"]);
  }
  
  if (myObject.hasOwnProperty("id")) {
    JSONVar jsonObject;
    jsonObject["id"] = "imu_gps";
    Serial.flush(); Serial.println(); // stop any messages
    Serial.println(jsonObject);    
    return true;
  }

  return false;
}

bool jsonParseCfg(JSONVar cfgObject) {
  bool retVal = false;
  if (cfgObject.hasOwnProperty("imu")) {
    g_imuEna = (bool) cfgObject["imu"];
    retVal = true;
  }
  if (cfgObject.hasOwnProperty("gps")) {
    g_gpsEna = (bool) cfgObject["gps"];
    retVal = true;
  }
  if (cfgObject.hasOwnProperty("cmp")) {
    g_cmpEna = (bool) cfgObject["cmp"];
    retVal = true;
  }
  return retVal;
}

void procCmp(void) {
  if(g_cmpEna == false) return;

  int a;
  
  // Read compass values
  compass.read();

  // Return Azimuth reading
    JSONVar jsonObject;
    jsonObject["cmp"]["azi"] = compass.getAzimuth();

    Serial.println(jsonObject);
}

// process the serial port connected to the uBlox GNSS M10Q
// NOTE: default is 1 msg/sec and 9600 baud
void procGps() {
  if (g_gpsEna == false) return;

  // Check to see if data is available

  if (myGNSS.getPVT()) {
    JSONVar jsonObject;
    jsonObject["gps"]["lat"] = myGNSS.getLatitude();
    jsonObject["gps"]["lon"] = myGNSS.getLongitude();
    jsonObject["gps"]["alt"] = myGNSS.getAltitude();
    jsonObject["gps"]["siv"] = myGNSS.getSIV();
    Serial.println(jsonObject);
  }
}

void procImu(void) {
  if (g_imuEna == false) return;

  if (bno08x.wasReset()) {
    Serial.print("sensor was reset ");
    setReports();
  }
  
  if (bno08x.getSensorEvent(&sensorValue)) {
    switch (sensorValue.sensorId) {
      case SH2_GYROSCOPE_CALIBRATED: //0x02
        json_SH2_GYROSCOPE_CALIBRATED(sensorValue);
        break;
      case SH2_LINEAR_ACCELERATION:  //0x04
        json_SH2_LINEAR_ACCELERATION(sensorValue);
        break;
      case SH2_ARVR_STABILIZED_RV:   // 0x28
        json_SH2_ARVR_STABILIZED_RV(sensorValue);
        break;
    }
  }
}

void json_SH2_GYROSCOPE_CALIBRATED(sh2_SensorValue_t sensorValue) {
  JSONVar jsonObject;
  jsonObject["imu"]["rvel"]["seq"]  = sensorValue.sequence;
  jsonObject["imu"]["rvel"]["stat"] = sensorValue.status;
  jsonObject["imu"]["rvel"]["x"]    = sensorValue.un.gyroscope.z;
  jsonObject["imu"]["rvel"]["y"]    = sensorValue.un.gyroscope.y;
  jsonObject["imu"]["rvel"]["z"]    = sensorValue.un.gyroscope.x;
  Serial.println(jsonObject);
}

void json_SH2_LINEAR_ACCELERATION(sh2_SensorValue_t sensorValue) {
  JSONVar jsonObject;
  jsonObject["imu"]["lacc"]["seq"] = sensorValue.sequence;
  jsonObject["imu"]["lacc"]["stat"] = sensorValue.status;
  jsonObject["imu"]["lacc"]["x"] = sensorValue.un.linearAcceleration.x;
  jsonObject["imu"]["lacc"]["y"] = sensorValue.un.linearAcceleration.y;
  jsonObject["imu"]["lacc"]["z"] = sensorValue.un.linearAcceleration.z;
  Serial.println(jsonObject);
}

void json_SH2_ARVR_STABILIZED_RV(sh2_SensorValue_t sensorValue) {
  JSONVar jsonObject;
  jsonObject["imu"]["rvec"]["seq"] = sensorValue.sequence;
  jsonObject["imu"]["rvec"]["stat"] = sensorValue.status;
  jsonObject["imu"]["rvec"]["i"] = sensorValue.un.arvrStabilizedRV.i;
  jsonObject["imu"]["rvec"]["j"] = sensorValue.un.arvrStabilizedRV.j;
  jsonObject["imu"]["rvec"]["k"] = sensorValue.un.arvrStabilizedRV.k;
  jsonObject["imu"]["rvec"]["real"] = sensorValue.un.arvrStabilizedRV.real;
  Serial.println(jsonObject);
}