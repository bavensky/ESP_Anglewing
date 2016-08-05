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
float p_angle, r_angle, rollAcc, pitchAcc, gyroXrate;
float P_CompCoeff = 0.95;
unsigned long time_prev;

void readMPU();
void wing_swing();

void setup() {
  Wire.begin();
  Serial.begin(38400);

  accelgyro.initialize();

  L_wing.attach(14);
  R_wing.attach(12);
  delay(1000);
    readMPU();

}

void loop() {
  readMPU();

  L_wing.write(90); //center
  R_wing.write(90); //center

  //  L_wing.write(130); //forward
  //  R_wing.write(50); //forward
  //  L_wing.write(50); //backward
  //  R_wing.write(130); //backward

  while (p_angle > 20 && p_angle < 30 ) {
    readMPU();
    L_wing.write(130);
    R_wing.write(50);
    delay(200);
    L_wing.write(50);
    R_wing.write(130);
    delay(200);
  }
  while (r_angle > 100 && r_angle < 110) {
    readMPU();
    L_wing.write(130);
    R_wing.write(90);
    delay(200);
    L_wing.write(50);
    R_wing.write(90);
    delay(200);
  }
  while (r_angle > 40 && r_angle < 50) {
    readMPU();
    L_wing.write(90);
    R_wing.write(50);
    delay(200);
    L_wing.write(90);
    R_wing.write(130);
    delay(200);
  }
}

void readMPU()  {
  unsigned long time_now = millis();
  if (time_now - time_prev >= 10)
  {
    time_prev = time_now;
    accelgyro.getMotion6(&ax, &ay, &az, &gx, &gy, &gz);

    p_angle += (((float)gz / 16.4f) * (-0.01f));
    float pitchAcc = atan2(ax, ay) * RAD_TO_DEG; //(float)22/7; //RAD_TO_DEG
    p_angle = P_CompCoeff * p_angle + (1.0f - P_CompCoeff) * pitchAcc;

    r_angle += ((gx / 16.4f) * (0.01f));
    float rollAcc = atan2(ay, az) * RAD_TO_DEG;
    r_angle = P_CompCoeff * r_angle + (1.0f - P_CompCoeff) * rollAcc;

//    Serial.print(r_angle);
//    Serial.print("\t");
//    Serial.println(p_angle);
  }
}
void wing_swing()  {
  L_wing.write(0);
  R_wing.write(0);
}
void wing_Lswing()  {
  L_wing.write(0);
  R_wing.write(0);
}
void wing_Rswing()  {
  L_wing.write(0);
  R_wing.write(0);
}

