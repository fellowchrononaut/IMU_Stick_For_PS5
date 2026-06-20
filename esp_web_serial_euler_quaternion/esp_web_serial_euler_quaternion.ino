/**
 * Simple example with bosch library for bno055 to work with
 * Adafruit webGL example with euler angles
 * https://adafruit.github.io/Adafruit_WebSerial_3DModelViewer/
 *
 * by Renzo Mischianti <www.mischianti.org>
 *
 * https://mischianti.org/
 */
 
#include "BNO055_support.h"     //Contains the bridge code between the API and Arduino
#include <Wire.h>
 
//The device address is set to BNO055_I2C_ADDR2 in this example. You can change this in the BNO055.h file in the code segment shown below.
// /* bno055 I2C Address */
// #define BNO055_I2C_ADDR1                0x28
// #define BNO055_I2C_ADDR2                0x29
// #define BNO055_I2C_ADDR                 BNO055_I2C_ADDR2
 
//Pin assignments as tested on the Arduino Due.
//Vdd,Vddio : 3.3V
//GND : GND
//SDA/SCL : SDA/SCL
//PSO/PS1 : GND/GND (I2C mode)
 
//This structure contains the details of the BNO055 device that is connected. (Updated after initialization)
struct bno055_t myBNO;
struct bno055_euler myEulerData; //Structure to hold the Euler data
struct bno055_quaternion myQuatData; //Structure to hold the Quaternion data
 
unsigned char accelCalibStatus = 0;     //Variable to hold the calibration status of the Accelerometer
unsigned char magCalibStatus = 0;       //Variable to hold the calibration status of the Magnetometer
unsigned char gyroCalibStatus = 0;      //Variable to hold the calibration status of the Gyroscope
unsigned char sysCalibStatus = 0;       //Variable to hold the calibration status of the System (BNO055's MCU)
 
unsigned long lastTime = 0;
#define DAC_X 25 //Ring 1 on TRRS
#define DAC_Y 26 //Tip on TRRS
float baseline_pitch = 0.0; // Baseline pitch (neutral)
float baseline_roll = 0.0; // Baseline roll (neutral)
float baseline_heading = 0.0; // Baseline heading (neutral)
bool is_calibrated = false;
 
/* Set the delay between fresh samples */
#define BNO055_SAMPLERATE_DELAY_MS (100)
#define MAX_ANGLE 25.0
 
void setup() //This code is executed once
{
  Serial.begin(115200);

  //Initialize I2C communication
  Wire.begin();

  // BNO055 needs ~650 ms after power-on before it accepts commands
  delay(800);

  //Initialization of the BNO055
  BNO_Init(&myBNO); //Assigning the structure to hold information about the device

  //Configuration to NDoF mode
  bno055_set_operation_mode(OPERATION_MODE_NDOF);

  // NDOF mode switch settle time (datasheet: 7-19 ms)
  delay(30);

  dacWrite(DAC_X, 128);
  dacWrite(DAC_Y, 128);
  Serial.println("IMU to DAC Initialized. Keep foot/sensor still for initial calibration...");
}

int mapAngleToDAC(float current, float base, float max_deflection, bool isHeading = false) {
  float delta = current - base;
  if (isHeading) {
    if (delta > 180.0) delta -= 360.0;
    else if (delta < -180.0) delta += 360.0;
  }
  // Clamp delta to max_angle range
  if (delta > max_deflection) delta = max_deflection;
  if (delta < -max_deflection) delta = -max_deflection;
  
  // Map -max_deflection..max_deflection to 0..255
  // Neutral (0 delta) becomes 127/128
  float normalized = (delta / max_deflection); // -1.0 to 1.0
  int dac_val = (int)((normalized + 1.0) * 127.5);
  return dac_val;
}
 
void loop() //This code is looped forever
{
  if ((millis() - lastTime) >= BNO055_SAMPLERATE_DELAY_MS) //To stream at 10Hz without using additional timers
  {
    lastTime = millis();
 
    bno055_read_euler_hrp(&myEulerData);            //Update Euler data into the structure

    float pitch = (float)myEulerData.p / 16.00;
    float roll  = (float)myEulerData.r / 16.00;
    float heading  = (float)myEulerData.h / 16.00;

    if (!is_calibrated && millis() > 2000) {
      baseline_pitch = pitch;
      baseline_roll = roll;
      baseline_heading = heading;
      is_calibrated = true;
      Serial.println("Neutral stance captured!");
    }

    if (is_calibrated) {
      // Bench-verified on this mounting: foot yaw drives BNO heading (Ring 1 on TRRS), foot pitch drives BNO roll (Tip on TRRS).
      // Stick X follows foot yaw, stick Y follows foot pitch.
      int dac_x_val = mapAngleToDAC(heading, baseline_heading, MAX_ANGLE, true);
      int dac_y_val = mapAngleToDAC(roll, baseline_roll, MAX_ANGLE, false);

      // Output to PS5 Access Controller
      dacWrite(DAC_X, dac_x_val);
      dacWrite(DAC_Y, dac_y_val);

      Serial.print("DAC-Attitude: ");
      Serial.print(heading); Serial.print(", ");
      Serial.print(pitch); Serial.print(", ");
      Serial.print(roll);
      Serial.print(" | DAC X: "); Serial.print(dac_x_val);
      Serial.print(" Y: "); Serial.println(dac_y_val);
    }


    bno055_read_quaternion_wxyz(&myQuatData);  // Update quaternion data

 
    /* The WebSerial 3D Model Viewer expects data as heading , pitch, roll */
    Serial.print(F("Orientation: "));
    Serial.print(360-(float(myEulerData.h) / 16.00));
    Serial.print(F(", "));
    Serial.print(360-(float(myEulerData.p) / 16.00));
    Serial.print(F(", "));
    Serial.print(360-(float(myEulerData.r) / 16.00));
    Serial.println(F(""));

    // Quaternion output (sensor gives fixed-point, scale = 1/16384)
    float qw = (float)myQuatData.w / 16384.0f;
    float qx = (float)myQuatData.x / 16384.0f;
    float qy = (float)myQuatData.y / 16384.0f;
    float qz = (float)myQuatData.z / 16384.0f;

    float norm = sqrt(qw*qw + qx*qx + qy*qy + qz*qz);
    if (norm > 0.0f) {
      qw /= norm;
      qx /= norm;
      qy /= norm;
      qz /= norm;
    }

    Serial.print(F("Quaternion: "));
    Serial.print(qw, 6);
    Serial.print(F(", "));
    Serial.print(qx, 6);
    Serial.print(F(", "));
    Serial.print(qy, 6);
    Serial.print(F(", "));
    Serial.print(qz, 6);
    Serial.println();

    // Euler-from-quaternion (ZYX intrinsic: yaw, pitch, roll) in degrees, yaw normalized to 0-360
    float sinp = 2.0f * (qw * qy - qz * qx);
    if (sinp >  1.0f) sinp =  1.0f;
    if (sinp < -1.0f) sinp = -1.0f;
    float yaw_q   = atan2(2.0f * (qw * qz + qx * qy), 1.0f - 2.0f * (qy * qy + qz * qz)) * 180.0f / PI;
    float pitch_q = asin(sinp) * 180.0f / PI;
    float roll_q  = atan2(2.0f * (qw * qx + qy * qz), 1.0f - 2.0f * (qx * qx + qy * qy)) * 180.0f / PI;
    if (yaw_q < 0.0f) yaw_q += 360.0f;

    Serial.print(F("EulerFromQuat: "));
    Serial.print(yaw_q, 2);   Serial.print(F(", "));
    Serial.print(pitch_q, 2); Serial.print(F(", "));
    Serial.println(roll_q, 2);

    bno055_get_accelcalib_status(&accelCalibStatus);
    bno055_get_gyrocalib_status(&gyroCalibStatus);
    bno055_get_syscalib_status(&sysCalibStatus);
    bno055_get_magcalib_status(&magCalibStatus);
 
    Serial.print(F("Calibration: "));
    Serial.print(sysCalibStatus, DEC);
    Serial.print(F(", "));
    Serial.print(gyroCalibStatus, DEC);
    Serial.print(F(", "));
    Serial.print(accelCalibStatus, DEC);
    Serial.print(F(", "));
    Serial.print(magCalibStatus, DEC);
    Serial.println(F(""));
 
 
  }
}
