#include "I2Cdev.h"
#include "MPU6050.h"
#include "Wire.h"
#include <Servo.h>

Servo L_wing;
Servo R_wing;

MPU6050 accelgyro;
#define OUTPUT_READABLE_ACCELGYRO
#define BT 0

int16_t modeCount = 0;
int16_t ax, ay, az;
int16_t gx, gy, gz;
unsigned long sum_x, sum_y;

int16_t ax_diff = -4040;
int16_t ay_diff = 25;
int16_t gx_diff = 15;
int16_t gy_diff = 15;
int16_t gz_diff = -3;

float p_angle, r_angle, rollAcc, pitchAcc, gyroXrate;
float P_CompCoeff = 0.95;

unsigned long time_prev = 0;
unsigned long previousMode1 = 0;
unsigned long previousMode2 = 0;
unsigned long previousMode3 = 0;


void readMPU();
void wing_swing();
void mode1();
void mode2();
void mode3();
void mode4();

void setup() {
  Wire.begin();
  Serial.begin(115200);

  accelgyro.initialize();
  pinMode(BT, INPUT);
  L_wing.attach(14);
  R_wing.attach(12);
  delay(1000);
}

void loop() {
  readMPU();
//  mode1();
//  mode2();
//  mode3();
  mode4();

  //  if(digitalRead(BT) == 0 )  {
  //    delay(200);
  //    modeCount++;
  //  }
  //
  //  if(modeCount == 1)  mode1();
  //  if(modeCount == 2)  mode2();
  //  if(modeCount == 3)  mode3();
  //  if(modeCount == 4)  modeCount = 0;

  // moveing wing
  //  L_wing.write(90); //wing left center
  //  R_wing.write(90); //wing right center
  //  L_wing.write(130); //forward
  //  R_wing.write(50); //forward
  //  L_wing.write(50); //backward
  //  R_wing.write(130); //backward
  //
  //  while (p_angle <= -20 && p_angle >= -40 ) {
  //    readMPU();
  //    L_wing.write(50);
  //    delay(100);
  //    L_wing.write(130);
  //    delay(100);
  //  }

  //  while (p_angle < -20 || p_angle > -40 ) {
  //    readMPU();
  //    L_wing.write(180);
  //    R_wing.write(50);
  //    delay(10);
  //    L_wing.write(50);
  //    R_wing.write(130);
  //    delay(200);
  //  }

  //  while (p_angle > 20 && p_angle < 30 ) {
  //    readMPU();
  //    L_wing.write(130);
  //    R_wing.write(50);
  //    delay(200);
  //    L_wing.write(50);
  //    R_wing.write(130);
  //    delay(200);
  //  }
  //  while (r_angle > 100 && r_angle < 110) {
  //    readMPU();
  //    L_wing.write(130);
  //    R_wing.write(90);
  //    delay(200);
  //    L_wing.write(50);
  //    R_wing.write(90);
  //    delay(200);
  //  }
  //  while (r_angle > 40 && r_angle < 50) {
  //    readMPU();
  //    L_wing.write(90);
  //    R_wing.write(50);
  //    delay(200);
  //    L_wing.write(90);
  //    R_wing.write(130);
  //    delay(200);
  //  }
}

void readMPU()  {
  unsigned long time_now = millis();
  if (time_now - time_prev >= 10)
  {
    time_prev = time_now;
    accelgyro.getMotion6(&ax, &ay, &az, &gx, &gy, &gz);

    // EX. set zero point
    //    rawAccx_X -= ax_diff;
    //    rawAccx_Y -= ay_diff;
    //    rawGyrox_X -= gx_diff;
    //    rawGyrox_Y -= gy_diff;
    //    rawGyrox_Z -= gz_diff;

    //    ax -= ax_diff;
    //    ay -= ay_diff;
    //
    //    gx -= gx_diff;
    //    gy -= gy_diff;
    //    gz -= gz_diff;

    //
    //    ax = sum_x/100;
    //    ay = sum_y/100;

    //    p_angle += (((float)gz / 16.4f)) * (-0.01f);
    //    p_angle += ((gz / 16.4f)) * (-0.01f);
    //    Serial.printf("Read IMU %d %d %d\n", ax, ay, gz);


    /*  default set mpu
        p_angle += ((gz / 16.4f)) * (0.01f);    // gz จุดที่เปลี่ยนตามแกนหมุน
        //(float)22/7; //RAD_TO_DEG แปลงเรเดียนเป็นองศา
        float pitchAcc = atan2(-ay, -ax) * RAD_TO_DEG; // atan2(1, 2)  1 คือจุดหมุน, 2 คือจุดตาม
        p_angle = P_CompCoeff * p_angle + (1.0f - P_CompCoeff) * pitchAcc;

        r_angle += ((-gy / 16.4f)) * (0.01f);
        float rollAcc = atan2(az, -ax) * RAD_TO_DEG;
        r_angle = P_CompCoeff * r_angle + (1.0f - P_CompCoeff) * rollAcc;
    */

    p_angle += ((gy / 16.4f)) * (0.01f);
    float pitchAcc = atan2(-ay, -az) * RAD_TO_DEG;
    p_angle = P_CompCoeff * p_angle + (1.0f - P_CompCoeff) * pitchAcc;

    r_angle += ((gx / 16.4f)) * (0.01f);
    float rollAcc = atan2(ax, -az) * RAD_TO_DEG;
    r_angle = P_CompCoeff * r_angle + (1.0f - P_CompCoeff) * rollAcc;

    Serial.print(p_angle);
    Serial.print("\t");
    Serial.println(r_angle);

  }
}

void mode1()  {
  unsigned long timenow = millis();
  readMPU();

  if (p_angle <= -7 && p_angle >= -40)  {
    if (timenow - previousMode1 <= 500) {
      L_wing.write(150);
      delay(250);
      L_wing.write(90);
      delay(250);
      L_wing.write(150);
      delay(250);
      L_wing.write(90);
      delay(250);
    }
    if (timenow - previousMode1 >= 3000) {
      previousMode1 = timenow;
    }
  } else {
    previousMode1 = timenow;
  }
}

void mode2()  {
  unsigned long timenow = millis();
  readMPU();

  if (p_angle <= 0 && p_angle >= -15)  {
    if (timenow - previousMode2 >= 250) {
      L_wing.write(50);
      R_wing.write(130);
    }
    if (timenow - previousMode2 >= 500) {
      L_wing.write(90);
      R_wing.write(90);
      previousMode2 = timenow;
    }
  }  else {
    previousMode2 = timenow;
  }
}

void mode3()  {
  unsigned long timenow = millis();
  readMPU();

  if (p_angle <= -31 && p_angle >= -40)  {
    if (timenow - previousMode3 >= 500) {
      L_wing.write(150);
    }
    if (timenow - previousMode3 >= 1000) {
      L_wing.write(90);
      previousMode3 = timenow;
    }
  }  else {
    previousMode3 = timenow;
  }
}

void mode4()  {
  readMPU();

  if (r_angle <= -10 && p_angle >= -20)  {
    R_wing.write(50);
  } else {
    R_wing.write(90);
  }
  if (r_angle >= 10 && p_angle <= 20)  {
    L_wing.write(130);
  } else {
    L_wing.write(90);
  }
}
