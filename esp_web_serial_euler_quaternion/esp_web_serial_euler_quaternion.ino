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
#define DAC_X 25
#define DAC_Y 26
float baseline_pitch = 0.0; // Baseline pitch (neutral)
float baseline_roll = 0.0; // Baseline roll (neutral)
float baseline_yaw = 0.0; // Baseline yaw (neutral)
bool is_calibrated = false;
 
/* Set the delay between fresh samples */
#define BNO055_SAMPLERATE_DELAY_MS (100)
#define MAX_ANGLE 25.0
 
void setup() //This code is executed once
{
  //Initialize I2C communication
  Wire.begin();
 
  //Initialization of the BNO055
  BNO_Init(&myBNO); //Assigning the structure to hold information about the device
 
  //Configuration to NDoF mode
  bno055_set_operation_mode(OPERATION_MODE_NDOF);
 
  delay(1);
 
  //Initialize the Serial Port to view information on the Serial Monitor
  Serial.begin(115200);
  dacWrite(DAC_X, 128);
  dacWrite(DAC_Y, 128);
  Serial.println("IMU to DAC Initialized. Keep foot/sensor still for initial calibration...");

}

int mapAngleToDAC(float current, float base, float max_deflection) {
  float delta = current - base;
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
    float yaw  = (float)myEulerData.h / 16.00;

    if (!is_calibrated && millis() > 2000) {
      baseline_pitch = pitch;
      baseline_roll = roll;
      baseline_yaw = yaw;
      is_calibrated = true;
      Serial.println("Neutral stance captured!");
    }

    if (is_calibrated) {
      // Map Roll to X-axis and Pitch to Y-axis
      int dac_x_val = mapAngleToDAC(roll, baseline_roll, MAX_ANGLE);
      int dac_y_val = mapAngleToDAC(yaw, baseline_yaw, MAX_ANGLE);

      // Output to PS5 Access Controller
      dacWrite(DAC_X, dac_x_val);
      dacWrite(DAC_Y, dac_y_val);

      Serial.print("Orientation: ");
      Serial.print(yaw); Serial.print(", ");
      Serial.print(pitch); Serial.print(", ");
      Serial.print(roll);
      Serial.print(" | DAC X: "); Serial.print(dac_x_val);
      Serial.print(" Y: "); Serial.println(dac_y_val);
    }


    bno055_read_quaternion_wxyz(&myQuatData);  // Update quaternion data

 
    /* The WebSerial 3D Model Viewer expects data as yaw , pitch, roll */
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
