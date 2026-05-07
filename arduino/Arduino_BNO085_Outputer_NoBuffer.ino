// Basic demo for readings from Adafruit BNO08x
#include <Wire.h>
#include <Adafruit_BNO08x.h>
#include <SparkFun_u-blox_GNSS_Arduino_Library.h>

// For SPI mode, we need a CS pin
#define BNO08X_CS 10
#define BNO08X_INT 9



// For SPI mode, we also need a RESET
//#define BNO08X_RESET 5
// but not for I2C or UART
#define BNO08X_RESET -1


SFE_UBLOX_GNSS myGNSS;
Adafruit_BNO08x bno08x(BNO08X_RESET);
sh2_SensorValue_t sensorValue;

    // Initializing IMU
  float accel_x         =   0;
  float accel_y         =   0;
  float accel_z         =   0;

  float gyro_x         =   0;
  float gyro_y         =   0;
  float gyro_z         =   0;

  float magn_x         =   0;
  float magn_y         =   0;
  float magn_z         =   0;

  static float dt             =   10;

  bool magn_updated  = false;

  unsigned long timestamp_ms = 0;

  
int32_t latitude = 0;
int32_t longitude =0;
int32_t altitude = 0;


void Disp_Rot_Calc(int acel_x, int acel_y, int acel_z, int accel_count, int gyo_x, int gyo_y, int gyo_z, int gyro_count) {

  
}

void setup(void) {

  Serial.begin(115200);
  while (!Serial)
    delay(10); // will pause Zero, Leonardo, etc until serial console opens
    
  Serial.println("Adafruit BNO08x test!");
  Wire.begin();
  Wire.setClock(400000);
  // Try to initialize!
  if(!myGNSS.begin()){
    Serial.println(F("GNSS not detected"));
    while (1) {
      delay(10);
    }
  } 

  Serial.println(F("GNSS successfully detected"));
  myGNSS.setI2COutput(COM_TYPE_UBX); //Set the I2C port to output UBX only (turn off NMEA noise)
   myGNSS.setAutoPVT(true);  

  if (!bno08x.begin_I2C()) {
    // if (!bno08x.begin_UART(&Serial1)) {  // Requires a device with > 300 byte
    // UART buffer! if (!bno08x.begin_SPI(BNO08X_CS, BNO08X_INT)) {
    Serial.println("Failed to find BNO08x chip");
    while (1) {
      delay(10);
    }
  }
 
  Serial.println("BNO08x Found!");


  Serial.println("Finding ublox");

  for (int n = 0; n < bno08x.prodIds.numEntries; n++) {
    Serial.print("Part ");
    Serial.print(bno08x.prodIds.entry[n].swPartNumber);
    Serial.print(": Version :");
    Serial.print(bno08x.prodIds.entry[n].swVersionMajor);
    Serial.print(".");
    Serial.print(bno08x.prodIds.entry[n].swVersionMinor);
    Serial.print(".");
    Serial.print(bno08x.prodIds.entry[n].swVersionPatch);
    Serial.print(" Build ");
    Serial.println(bno08x.prodIds.entry[n].swBuildNumber);
  }
  

 //myGNSS.enableDebugging(); // Uncomment this line to enable helpful debug messages on Serial
  
  setReports();

  Serial.println("Reading events");
  delay(100);
}

// Here is where you define the sensor outputs you want to receive
void setReports(void) {
  Serial.println("Setting desired reports");
  if (!bno08x.enableReport(SH2_ACCELEROMETER)) {
    Serial.println("Could not enable accelerometer");
  }
  if (!bno08x.enableReport(SH2_GYROSCOPE_CALIBRATED)) {
    Serial.println("Could not enable gyroscope");
  }
  if (!bno08x.enableReport(SH2_MAGNETIC_FIELD_CALIBRATED)) {
    Serial.println("Could not enable magnetic field calibrated");
  }
 
}

void loop() {

  if (bno08x.wasReset()) {
    setReports();
  }

  if (!bno08x.getSensorEvent(&sensorValue)) {
    return;
  }

  switch (sensorValue.sensorId) {
    
      
    case SH2_ACCELEROMETER:
      accel_x = sensorValue.un.accelerometer.x;
      accel_y = sensorValue.un.accelerometer.y;
      accel_z = sensorValue.un.accelerometer.z;
      
      break;

    case SH2_GYROSCOPE_CALIBRATED:
      gyro_x = sensorValue.un.gyroscope.x;
      gyro_y = sensorValue.un.gyroscope.y;
      gyro_z = sensorValue.un.gyroscope.z;
      
      break;

    case SH2_MAGNETIC_FIELD_CALIBRATED:

      // Store mag (single sample)
      magn_x = sensorValue.un.magneticField.x;
      magn_y = sensorValue.un.magneticField.y;
      magn_z = sensorValue.un.magneticField.z;
      
      break;

    
    default:
    break;
  }
      // Only output if we actually collected accel & gyro samples
        // Compute averages
        
        if(myGNSS.getPVT()){
        latitude = myGNSS.getLatitude();
        longitude = myGNSS.getLongitude();
        altitude = myGNSS.getAltitudeMSL(); // Altitude above Mean Sea Level
        }   
        
        
        

        timestamp_ms = millis();

        Serial.print(timestamp_ms);
        Serial.print(", ");

        Serial.print(accel_x); Serial.print(", ");
        Serial.print(accel_y); Serial.print(", ");
        Serial.print(accel_z); Serial.print(", ");

        Serial.print(gyro_x);  Serial.print(", ");
        Serial.print(gyro_y);  Serial.print(", ");
        Serial.print(gyro_z);  Serial.print(", ");

        
        Serial.print(magn_x);  Serial.print(", ");
        Serial.print(magn_y);  Serial.print(", ");
        Serial.print(magn_z);Serial.print(", ");
       
        Serial.print(latitude);Serial.print(", ");
        Serial.print(longitude); //(degrees * 10^-7)
        Serial.print(", ");
        Serial.println(altitude);//(mm)

            // Reset accumulators for next mag interval
      


}
