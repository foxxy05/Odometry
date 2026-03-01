// Teensy / Arduino
#include <Wire.h>
#define slaveAddr 0X08
#define dataBuffer 10

#define TEENSY 13
#include <math.h>
#include <Encoder.h>
#include <BTS7960.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BNO055.h>
#include <utility/imumaths.h>

// CONSTANTS 
#define perpendicularOffset         0.200
#define parallelOffset              0.200
#define wheelRadius                 0.075
#define maxPWM                      50
#define PI                          3.1415962

#define sqrt3by2                    0.8660254038
#define minus1by2                  -0.5000
#define constVector                 1
#define L                           1

#define arraySize 4

// ENCODERS 
Encoder FEnc(5, 6);
Encoder LEnc(7, 8);
Encoder REnc(9, 10);

double CPR = 8192.0;

double FCount = 0, LCount = 0, RCount = 0;
double prevFCount = 0, prevLCount = 0, prevRCount = 0;
double dF, dL, dR;

// POSITION 
double xPosition = 0;
double yPosition = 0;
double thetaPosition = 0;

double deltaX, deltaY;
double deltaX_global, deltaY_global;

double prevTheta = 0;   // IMU previous heading

// MOTORS 
BTS7960 FW(12, 11);
BTS7960 LW(9, 10);
BTS7960 RW(5, 6);

int16_t wFW = 0, wLW = 0, wRW = 0;
int16_t Vx = 0, Vy = 0;
int16_t VxG = 0, VyG = 0;
int16_t omega = 0;

// PS4 
int8_t receivedData[arraySize] = {0};

// PID 
unsigned long currentTime = 0, previousTime = 0;
float error = 0, previousError = 0;
float derivative = 0;
float kp = 8.0;
float kd = 68;
float PID = 0;

//  IMU 
Adafruit_BNO055 bno = Adafruit_BNO055();
int targetAngle = 0;
int currentAngle = 0;


void setup() {
  Serial.begin(115200);
  pinMode(TEENSY, OUTPUT);
  digitalWrite(TEENSY, HIGH);

  // Setting-up I communication between ESP32 and Arduino
  Wire2.begin();
  Serial.println("I2C Master Ready!");

  FW.setEnable(true);
  LW.setEnable(true);
  RW.setEnable(true);

  if (!bno.begin()) {
    while (1);
  }
  bno.setExtCrystalUse(true);
  delay(1000);

  // Initialize encoder reference
  prevFCount = FEnc.read();
  prevLCount = LEnc.read();
  prevRCount = REnc.read();

  // Initialize IMU reference
  imu::Vector<3> euler = bno.getVector(Adafruit_BNO055::VECTOR_EULER);
  prevTheta = euler.x() * PI / 180.0;
  thetaPosition = prevTheta;
  previousTime = millis();
}

// =============================================================

void loop() {
  imu::Vector<3> euler = bno.getVector(Adafruit_BNO055::VECTOR_EULER);
  currentAngle = euler.x();

  double trueTheta = currentAngle * PI / 180.0;

  // IMU DELTA THETA 
  double delTheta = trueTheta - prevTheta;

  if (delTheta > PI)  delTheta -= 2 * PI;
  if (delTheta < -PI) delTheta += 2 * PI;

  double midTheta = prevTheta + ( delTheta / 2.0 );

  // ENCODER READ 
  FCount = FEnc.read();
  LCount = LEnc.read();
  RCount = REnc.read();

  dF = FCount - prevFCount;
  dL = LCount - prevLCount;
  dR = RCount - prevRCount;

  // TRANSLATION 
  deltaX = (dF / CPR) * 2 * PI * wheelRadius;
  deltaY = (((dL + dR) / 2.0 )/ CPR) * 2 * PI * wheelRadius;

  deltaX += delTheta * perpendicularOffset;
  deltaY -= delTheta * parallelOffset;

  // GLOBAL TRANSFORM 
  deltaX_global= deltaX * cos(midTheta) - deltaY * sin(midTheta);
  deltaY_global = deltaX * sin(midTheta) + deltaY * cos(midTheta);

  xPosition += deltaX_global;
  yPosition += deltaY_global;
  thetaPosition = trueTheta;

  // PS4 
  receivePS4();

  Vy = receivedData[0];
  Vx = receivedData[1];
  omega = receivedData[2] - receivedData[3];

  VxG = Vx * cos(-thetaPosition) - Vy * sin(-thetaPosition);
  VyG = Vx * sin(-thetaPosition) + Vy * cos(-thetaPosition);

  // ANGULAR HOLD 
  if (abs(omega) < 3) {

    error = currentAngle - targetAngle;
    if (error >= 180) error -= 360;
    if (error < -180) error += 360;

    if (fabs(error) < 2) {
      omega = 0;
    } else {
      omega = -PIDControl(error);
    }

  } else {
    targetAngle = currentAngle;
  }

  // WHEEL EQUATIONS 
  wFW = constrain(constVector * (VxG*(minus1by2) + VyG*(sqrt3by2) + omega), -maxPWM, maxPWM);
  wLW = constrain(constVector * (VxG*(minus1by2) - VyG*(sqrt3by2) + omega), -maxPWM, maxPWM);
  wRW = constrain(constVector * (VxG + omega), -maxPWM, maxPWM);

  FW.rotate(wFW);
  RW.rotate(wRW);
  LW.rotate(wLW);

  // UPDATE PREVIOUS 
  prevFCount = FCount;
  prevLCount = LCount;
  prevRCount = RCount;
  prevTheta  = trueTheta;
}


void receivePS4() {
  Wire2.requestFrom(slaveAddr, sizeof(receivedData));

  int i = 0;
  while (Wire2.available()) {
    uint8_t raw = Wire2.read();
    if (i == 0) receivedData[0] = map(raw, 0, 255, -127, 127);       // LX
    else if (i == 1) receivedData[1] = map(raw, 0, 255, -127, 127);  // LY
    else if (i == 2) receivedData[2] = map(raw, 0, 255, 0, -127);    // L2
    else if (i == 3) receivedData[3] = map(raw, 0, 255, 0, 127);     // R2

    i++;
  }

  if (abs(receivedData[0]) < dataBuffer) receivedData[0] = 0;
  if (abs(receivedData[1]) < dataBuffer) receivedData[1] = 0;
  if (abs(receivedData[2]) < dataBuffer) receivedData[2] = 0;
  if (abs(receivedData[3]) < dataBuffer) receivedData[3] = 0;
}


float PIDControl(float error) {
  currentTime = millis();
  float deltaT = (currentTime - previousTime) / 1000.0;
  if (deltaT <= 0) deltaT = 0.001;

  derivative = (error - previousError) / deltaT;
  PID = kp * error + kd * derivative;

  previousError = error;
  previousTime = currentTime;

  PID = constrain(PID, -maxPWM, maxPWM);
  if (fabs(PID) <= 1) PID = 0;

  return PID;
}