#include <Servo.h>
#include <Wire.h>
#include <Adafruit_BNO08x.h>

// For SPI mode, we also need a RESET
//#define BNO08X_RESET 5
// but not for I2C or UART
#define BNO08X_RESET -1


// SFE_UBLOX_GNSS myGNSS;
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


// encoder globals

// --- PIN DEFINITIONS ---
// Encoder pins for Mega 2560 (Must be interrupt-capable)
const int L_ENC_A = 3;   const int L_ENC_B = 2;  
const int R_ENC_A = 18;  const int R_ENC_B = 19;

// PWM pins for Spark Max (controlled via Servo library)
const int L_SPARK_PIN = 8;
const int R_SPARK_PIN = 10;

// --- PHYSICAL & CONTROL CONSTANTS ---
const float TICKS_PER_REV = 9650.0;
const float TICKS_PER_RAD = TICKS_PER_REV / (2.0 * M_PI);

const int PWM_NEUTRAL = 1500;  // Spark Max mid-point
const int PWM_DEADBAND = 50;   // Minimum "kick" to overcome internal friction
const int PWM_LIMIT = 500;     // Max allowable deviation (1000us to 2000us)

// --- GAINS & TUNING ---
// Feed-Forward (kV): The "Base Power" per 1 rad/s.
// We set Right lower (80) because it is naturally faster than Left (88).
const float kV_L = 88.0; 
const float kV_R = 80.0; 

// PI Gains: Kp for instant reaction, Ki for long-term drift correction.
float Kp = 5.0;  
float Ki = 1.5;

// --- GLOBAL VARIABLES ---
volatile long l_ticks = 0;
volatile long r_ticks = 0;

float l_target = 0, r_target = 0;      // Target speeds from Jetson (rad/s)
float l_integral = 0, r_integral = 0;  // Accumulated error for Ki
long l_prev_ticks = 0, r_prev_ticks = 0;

unsigned long last_loop = 0;           // Timer for 100Hz loop
unsigned long last_cmd_time = 0;       // Timer for Watchdog safety

Servo l_motor, r_motor;

void setup() {
  Serial.begin(115200);
  
  // Setup Encoders with Pullups to prevent noise
  pinMode(L_ENC_A, INPUT_PULLUP); pinMode(L_ENC_B, INPUT_PULLUP);
  pinMode(R_ENC_A, INPUT_PULLUP); pinMode(R_ENC_B, INPUT_PULLUP);
  
  // Attach Interrupts: Trigger on ANY state change for maximum resolution
  attachInterrupt(digitalPinToInterrupt(L_ENC_A), l_encoder_isr, CHANGE);
  attachInterrupt(digitalPinToInterrupt(R_ENC_A), r_encoder_isr, CHANGE);
  
  // Initialize Motors
  l_motor.attach(L_SPARK_PIN); r_motor.attach(R_SPARK_PIN);
  l_motor.writeMicroseconds(PWM_NEUTRAL); r_motor.writeMicroseconds(PWM_NEUTRAL);

  while (!Serial)
    delay(10); // will pause Zero, Leonardo, etc until serial console opens
    
  Serial.println("Adafruit BNO08x test!");
  Wire.begin();
  Wire.setClock(400000);
  // // Try to initialize!
  // if(!myGNSS.begin()){
  //   Serial.println(F("GNSS not detected"));
  //   while (1) {
  //     delay(10);
  //   }
  // };  

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
  
  setReports();

  Serial.println("Reading events");
  delay(100);
  
  last_loop = millis();

}

void loop() {
  // 1. ROBUST SERIAL PARSING
  // We check for the 'm' header and use parseFloat to handle messy strings.
  if (Serial.available()) {
    if (Serial.peek() == 'm') {
      Serial.read(); // Remove 'm' from buffer
      l_target = Serial.parseFloat();
      r_target = Serial.parseFloat();
      last_cmd_time = millis(); // Refresh the safety watchdog
      
      // Flush the buffer to ensure we don't read old commands next time
      while(Serial.available() && Serial.read() != '\n'); 
    } else {
      Serial.read(); // Clear unknown characters
    }
  }
  if (bno08x.wasReset()) {
    setReports();
  }

  if (bno08x.getSensorEvent(&sensorValue)) {
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
  }
  // switch (sensorValue.sensorId) {
  //   case SH2_ACCELEROMETER:
  //     accel_x = sensorValue.un.accelerometer.x;
  //     accel_y = sensorValue.un.accelerometer.y;
  //     accel_z = sensorValue.un.accelerometer.z;
      
  //     break;

  //   case SH2_GYROSCOPE_CALIBRATED:
  //     gyro_x = sensorValue.un.gyroscope.x;
  //     gyro_y = sensorValue.un.gyroscope.y;
  //     gyro_z = sensorValue.un.gyroscope.z;
      
  //     break;

  //   case SH2_MAGNETIC_FIELD_CALIBRATED:

  //     // Store mag (single sample)
  //     magn_x = sensorValue.un.magneticField.x;
  //     magn_y = sensorValue.un.magneticField.y;
  //     magn_z = sensorValue.un.magneticField.z;
      
  //     break;

    
  //   default:
  //   break;
  // }
      // Only output if we actually collected accel & gyro samples
        // Compute averages
        
      // // if(myGNSS.getPVT()){
      //   latitude = myGNSS.getLatitude();
      //    longitude = myGNSS.getLongitude();
      //    altitude = myGNSS.getAltitudeMSL(); // Altitude above Mean Sea Level
      //    }   

  unsigned long now = millis();
  float dt = (now - last_loop) / 1000.0;

  // 2. FIXED FREQUENCY CONTROL LOOP (100Hz)
  if (dt >= 0.01) { 
    // Capture encoder counts safely (disable interrupts during 4-byte read)
    noInterrupts();
    long l_curr = l_ticks; long r_curr = r_ticks;
    interrupts();

    // SAFETY WATCHDOG
    // If no command is received for 0.5s, force target to zero.
    if (now - last_cmd_time > 500) { l_target = 0; r_target = 0; }

    // 3. VELOCITY CALCULATION
    float l_vel = ((l_curr - l_prev_ticks) / TICKS_PER_RAD) / dt;

      int dir = (l_target > 0) ? 1 : -1;
      l_pwm += (dir * PWM_DEADBAND) + (int)l_out;
    }
    if (abs(r_target) > 0.01) {
      int dir = (r_target > 0) ? 1 : -1;
      r_pwm += (dir * PWM_DEADBAND) + (int)r_out;
    }

    // Write constrained PWM to Spark Max
    l_motor.writeMicroseconds(constrain(l_pwm, 1000, 2000));
    r_motor.writeMicroseconds(constrain(r_pwm, 1000, 2000));

    // 6. TELEMETRY STREAM
    // Push cumulative Radian position to Jetson for Odometry calculation.
    

    // Save state for next 10ms loop
    l_prev_ticks = l_curr; r_prev_ticks = r_curr;
    last_loop = now;
  }
}

// --- ENCODER INTERRUPT SERVICE ROUTINES ---
// These ensure Forward = Positive for both sides by accounting for mirrored mounting.
void l_encoder_isr() {
  // Pin 3 is Bit 5 of PORTE. Pin 2 is Bit 4 of PORTE.
  bool a_state = (PINE & (1 << 5)); 
  bool b_state = (PINE & (1 << 4));
  
  if (a_state == b_state) l_ticks++; else l_ticks--;
}

void r_encoder_isr() {
  // Pin 18 is Bit 3 of PORTD. Pin 19 is Bit 2 of PORTD.
  bool a_state = (PIND & (1 << 3));
  bool b_state = (PIND & (1 << 2));
  
  // Swapped logic for the right motor since it faces the opposite direction
  if (a_state == b_state) r_ticks--; else r_ticks++; 
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


