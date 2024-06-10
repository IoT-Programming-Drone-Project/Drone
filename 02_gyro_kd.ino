#include <Wire.h>

void setup() {
  Serial1.begin(115200);

  Wire.begin();
  Wire.setClock(400000);

  Wire.beginTransmission(0x68);
  Wire.write(0x6b);
  Wire.write(0x0);
  Wire.endTransmission(true);

}

int throttle = 0;
void loop() {
  Wire.beginTransmission(0x68);
  Wire.write(0x43);
  Wire.endTransmission(false);
  Wire.requestFrom(0x68, 6, true);
  int16_t GyXH = Wire.read();
  int16_t GyXL = Wire.read();
  int16_t GyYH = Wire.read();
  int16_t GyYL = Wire.read();
  int16_t GyZH = Wire.read();
  int16_t GyZL = Wire.read();
  int16_t GyX = GyXH << 8 | GyXL;
  int16_t GyY = GyYH << 8 | GyYL;
  int16_t GyZ = GyZH << 8 | GyZL;

  static int32_t GyXSum = 0, GyYSum = 0, GyZSum = 0;
  static double GyXOff = 0.0, GyYOff = 0.0, GyZOff = 0.0;
  static int cnt_sample = 1000;
  if (cnt_sample > 0){
    GyXSum += GyX, GyYSum += GyY, GyZSum += GyZ;
    cnt_sample --;
    if (cnt_sample == 0){
      GyXOff = GyXSum / 1000.0;
      GyYOff = GyYSum / 1000.0;
      GyZOff = GyZSum / 1000.0;
    }
    delay(1);
    return;
  }
  double GyXD = GyX - GyXOff;
  double GyYD = GyY - GyYOff;
  double GyZD = GyZ - GyZOff;
  double GyXR = GyXD / 131;
  double GyYR = GyYD / 131;
  double GyZR = GyZD / 131;

  static unsigned long t_prev = 0;
  unsigned long t_now = micros();
  double dt = (t_now - t_prev)/1000000.0;
  t_prev = t_now;

  static double AngleX = 0.0, AngleY = 0.0, AngleZ = 0.0;
  AngleX += GyXR * dt;
  AngleY += GyYR * dt;
  AngleZ += GyZR * dt;
  if (throttle == 0) AngleX = AngleY = AngleZ = 0.0;

  static double tAngleX = 0.0, tAngleY = 0.0, tAngleZ = 0.0;
  double eAngleX = tAngleX - AngleX;
  double eAngleY = tAngleY - AngleY;
  double eAngleZ = tAngleZ - AngleZ;
  double Kp = 1.0;
  double BalX = Kp * eAngleX;
  double BalY = Kp * eAngleY;
  double BalZ = Kp * eAngleZ;

  double Kd = 1.0;
  BalX += Kd * -GyXR;
  BalY += Kd * -GyYR;
  BalZ += Kd * -GyZR;
  if (throttle == 0) BalX = BalY = BalZ = 0.0;

  if (Serial1.available()>0){
    while(Serial1.available()>0){
      char userInput = Serial1.read();
      if (userInput >= '0' && userInput <= '9'){
        throttle = (userInput - '0') * 25;
      }
    }
  }

  double speedA = throttle + BalY + BalX - BalZ;
  double speedB = throttle - BalY + BalX + BalZ;
  double speedC = throttle - BalY - BalX - BalZ;
  double speedD = throttle + BalY - BalX + BalZ;

  int iSpeedA = constrain((int)speedA, 0, 250);
  int iSpeedB = constrain((int)speedB, 0, 250);
  int iSpeedC = constrain((int)speedC, 0, 250);
  int iSpeedD = constrain((int)speedD, 0, 250);

  analogWrite(6, iSpeedA);
  analogWrite(10, iSpeedB);
  analogWrite(9, iSpeedC);
  analogWrite(5, iSpeedD);
}
