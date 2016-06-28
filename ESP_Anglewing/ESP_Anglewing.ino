#include "I2Cdev.h"
#include "MPU6050.h"
#include "Wire.h"
#include <Servo.h> 

Servo L_wing;
Servo R_wing;

MPU6050 accelgyro;
#define OUTPUT_READABLE_ACCELGYRO



int16_t ax, ay, az;
int16_t gx, gy, gz;
float f_angle, pitchAcc, gyroXrate;
float P_CompCoeff = 0.95;
int timer = 10;

void setup() {
  Wire.begin();
  Serial.begin(38400);
  
  accelgyro.initialize();

  L_wing.attach(14);
  R_wing.attach(12);
}
unsigned long time_prev;
void loop() {
  unsigned long time_now = millis();
  if (time_now - time_prev >= 10)
  {
    time_prev = time_now;
    accelgyro.getMotion6(&ax, &ay, &az, &gx, &gy, &gz);
    f_angle += (((float)gz / 16.4f) * (-0.01f));
    float pitchAcc = atan2(ax, -ay) * RAD_TO_DEG; //(float)22/7; //RAD_TO_DEG
    f_angle = P_CompCoeff * f_angle + (1.0f - P_CompCoeff) * pitchAcc;
    //    Serial.print("  f_angle = ");
    //    Serial.println(f_angle);
  }
  L_wing.write(0); 
  R_wing.write(0); 
}
