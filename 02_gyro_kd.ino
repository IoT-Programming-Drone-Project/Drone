#include <Wire.h>

char message;
unsigned long setupTime = 0;

void setup() {
  Serial.begin(9600);
  Serial1.begin(115200);
  setupTime = millis(); // setup() 함수가 호출된 시간 저장

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
    cnt_sample--;
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

  // static int cnt_loop;
  // cnt_loop++;
  // if (cnt_loop%100 != 0) return 0;
  // Serial.print("GyYD = "); Serial.print(GyYD);
  // Serial.println();

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

  double Ki = 0.5;
  static double ResX = 0.0, ResY = 0.0, ResZ = 0.0;
  ResX += Ki * eAngleX * dt;
  ResY += Ki * eAngleY * dt;
  ResZ += Ki * eAngleZ * dt;
  if (throttle == 0) ResX = ResY = ResZ = 0.0;
  BalX += ResX;
  BalY += ResY;
  BalZ += ResZ;

  unsigned long cTime = millis() - setupTime;
  
  long tt = 3000;

  tAngleY = -7;
  if (cTime < 3000) {
    throttle = 170;
  } else if (cTime < 5500) {
    throttle = 180;
    tAngleX = -20;
  } else if (cTime < 5700) {
    throttle = 170;
    tAngleX = +40;
    tAngleY = -3;
  } else if (cTime < 6500) {
    tAngleX = 0.0;
    throttle = 160;
  // } else if (cTime < 7500) {
  //   throttle = 110;
  // } else if (cTime < 8500) {
  //   throttle = 100;
  // } else if (cTime < 11000) {
  //   throttle = 100;
  // } else if (cTime < 12500) {
  //   throttle = 100;
  } else if (cTime < 9000) {
    throttle = 140;
  } else {
    throttle = 100;
  }

  // 블루투스 코드랍니다~!
  if (Serial1.available() > 0) {
    while(Serial1.available() > 0) {
      char userInput = Serial1.read();
      float k=3.0;
      
      if ('0' <= userInput && userInput <= '9') {
        throttle = (userInput - '0') * 25;
      } else if (userInput == 'a') { // 왼쪽으로 기울기 Roll -
        tAngleY -= k;
      } else if (userInput == 'd') { // 오른쪽으로 기울기 Roll +
        tAngleY += k;
      } else if (userInput == 'w') { // 앞으로 기울기 Pitch -
        tAngleX -= k;
      } else if (userInput == 'x') { // 뒤로 기울기 Pitch +
        tAngleX += k;
      } else if (userInput == 'q') { // 시계 방향 회전 Yaw -
        tAngleZ -= k;
      } else if (userInput == 'e') { // 반시계 방향 회전 Yaw +
        tAngleZ += k;
      } else if (userInput == 's') { // 제자리에 서기
        tAngleX = tAngleY = tAngleZ = 0.0;
      } else if (userInput == 'r') {
        analogWrite(6, 0);
        analogWrite(10, 0);
        analogWrite(9, 0);
        analogWrite(5, 0);
        delay(100000);
      }
    }
  }

  double speedA = throttle + BalY + BalX - BalZ;
  double speedB = throttle - BalY + BalX + BalZ;
  double speedC = throttle - BalY - BalX + 4 - BalZ;
  double speedD = throttle + BalY - BalX + 4 + BalZ;

  int iSpeedA = constrain((int)speedA, 0, 250);
  int iSpeedB = constrain((int)speedB, 0, 250);
  int iSpeedC = constrain((int)speedC, 0, 250);
  int iSpeedD = constrain((int)speedD, 0, 250);

  analogWrite(6, iSpeedA);
  analogWrite(10, iSpeedB);
  analogWrite(9, iSpeedC);
  analogWrite(5, iSpeedD);

  if (cTime % 1000 == 0) {
    Serial1.write("a");
    Serial.write("a");
  }
  
  
  // if (Serial1.available()){
  //   message = Serial1.read();
  //   Serial1.println(message);
  // }

  // static int cnt_loop;
  // cnt_loop ++;
  // if (cnt_loop%1000 !=0) return;

  // Serial.print("A = ");    Serial.print(speedA);
  // Serial.print(" | B = "); Serial.print(speedB);
  // Serial.print(" | C = "); Serial.print(speedC);
  // Serial.print(" | D = "); Serial.print(speedD);
  // Serial.println();

}


