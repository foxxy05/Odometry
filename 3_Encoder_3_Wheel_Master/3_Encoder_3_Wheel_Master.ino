// Teensy / Arduino
#define TEENSY 13
// Encoder Setup
#define CPR 8192
#define offset 0.200
#define T 0.200
#define R 0.125
Encoder FEnc(5, 6);
Encoder LEnc(7, 8);
Encoder REnc(9, 10);

double FCount = 0;
double LCount = 0;
double RCount = 0;

// Position setup
double prevFCount;
double prevLCount;
double prevRCount;

double dF;
double dL;
double dR;

double xPosition;
double yPosition;
double thetaPosition;

double midTheta;
double delX;
double delY;
double delTheta;
double meanEncCount;
double thetaMid;
double changeInX;
double changeInY;
double changeInTheta;

// BTS Setup
#include <BTS7960.h>
#define maxPWM 50
// Add proper pins for PWM and enable according to board of choice
// Directly add the PWM and enable pins when declaring the object
BTS7960 FW(12, 11);
BTS7960 LW(9, 10);
BTS7960 RW(5, 6);


// #define rad 0.55 // radius of the wheels
// #define constVector 0.70710  // 1/sqrt(2)
// #define constVector 1.28564
#define sqrt3by2 0.8660254038
#define minus1by2 -0.5000
#define constVector 1
#define L 1  // net-length (lx+ly)
// #define buffer 10
// #define constVector 1
#define arraySize 4  // LX  LY  L2  R2
int8_t receivedData[arraySize] = { 0 };

// Navigation Variables
int16_t wFW = 0, wLW = 0, wRW = 0;
int16_t Vx = 0, Vy = 0;
int16_t VxG = 0, VyG = 0;
int16_t omega = 0;

// //PID
float currentTime = 0;
float previousTime = 0;
float error = 0;
float previousError = 0;
float derivative = 0;
float kp = 8.0;
float kd = 68;
float PID = 0;
int targetAngle = 0;

// BNO
#include <Adafruit_Sensor.h>
#include <Adafruit_BNO055.h>
#include <utility/imumaths.h>
#define PI 3.1415962
int currentAngle = 0;

Adafruit_BNO055 bno = Adafruit_BNO055();

void setup() {
  Serial.begin(115200);
  Serial.println("I2C Master Ready!");
  pinMode(TEENSY, OUTPUT);
  digitalWrite(TEENSY, HIGH);

  // Setting-up UART communication between ESP32 and Arduino
  Serial1.begin(9600);
  Serial.println("UART1 is active!");

  // Setting the enable as HIGH for each BTS
  FW.setEnable(true);
  LW.setEnable(true);
  RW.setEnable(true);

  // Initiating BNO and setting extCrystal as true
  if (!bno.begin()) {
    // Serial.print("No BNO055 detected");
    bno.setExtCrystalUse(true);
    while (1)
      ;
  }
  delay(1000);
}

void loop() {
  imu::Vector<3> euler = bno.getVector(Adafruit_BNO055::VECTOR_EULER);
  currentAngle = 0;
  wFW = 0;
  wLW = 0;
  wRW = 0;
  Vx = 0, Vy = 0;
  VxG = 0, VyG = 0;
  omega = 0;

  FCount = FEnc.read();
  LCount = LEnc.read();
  RCount = REnc.read();

  currentAngle = euler.x();
  Serial.println(currentAngle);
  float theta = currentAngle * PI / 180.0;
  double trueTheta = theta;

  //Encoder calculations
  dF = FCount - prevFCount;
  dL = LCount - prevLCount;
  dR = RCount - prevRCount;

  meanEncCount = (dL + dR)/2;
  delX = (dF/CPR) *2 * PI * R;
  delY = (meanEncCount/CPR) * 2 * PI * R;
  delTheta = (dR - dL) * (2 * PI * R)/(CPR * T);
  midTheta  = trueTheta + (delTheta/2);
  delX = delX + (delTheta * offset);

  // Global coordinates
  changeInX = delX * cos(midTheta) - delY * sin(midTheta);
  changeInY = delX * sin(midTheta) + delY * cos(midTheta);
  changeInTheta = delTheta;

  xPosition = xPosition + changeInX;
  yPosition = yPosition + changeInY;
  thetaPosition = thetaPosition + changeInTheta;

  receivePS4();

  //Global Calculations
  Vy = receivedData[0];  //Y-Component of the Joystick is the X component of the Chassis
  Vx = receivedData[1];
  omega = receivedData[2] - receivedData[3];
  VxG = Vx * cos(-theta) - Vy * sin(-theta);  // Local X
  VyG = Vx * sin(-theta) + Vy * cos(-theta);  // Local Y

  if (abs(omega) < 10) {
    error = currentAngle - targetAngle;
    if (error > 180) error -= 360;
    if (error < -180) error += 360;

    // currentTime = millis();
    // int deltaT = (currentTime - previousTime);
    // if (deltaT <= 0) {
    //   deltaT = 1;
    // }
    // derivative = (error - previousError) / (deltaT);
    // PID = kp * error + kd * derivative;

    // PID = constrain(PID, -maxPWM, maxPWM);
    // if (abs(PID) <= 1) {
    //   PID = 0;
    // }
    omega = PIDControl(error);
    previousError = error;
    previousTime = currentTime;
  } 
  // else {
  //   targetAngle = currentAngle;
  // }

  // Front wheel (120d)
  wFW = constrain(constVector * (VxG*(minus1by2) + VyG*(sqrt3by2) + omega), -maxPWM, maxPWM);
  // Left Wheel (240d)
  wLW = constrain(constVector * (VxG*(minus1by2) - VyG*(sqrt3by2) + omega), -maxPWM, maxPWM);
  // Right Wheel (0d)
  wRW = constrain(constVector * (VxG - VyG*(0) + omega), -maxPWM, maxPWM);

  // Sending equation's values to BTS
  FW.rotate(wFW);
  RW.rotate(wRW);
  LW.rotate(wLW);

  prevFCount = FCount;
  prevLCount = LCount;
  prevRCount = RCount;
  // targetAngle = currentAngle;
  // printEq();
  // printPS();
  // delay(10);
}

void receivePS4() {
  static bool receiving = false;
  static int index = 0;

  while (Serial1.available()) {
    uint8_t b = Serial1.read();

    if (b == 0xFF) {  // Start marker
      receiving = true;
      index = 0;
    } else if (b == 0xFE && receiving) {  // End marker
      if (index == arraySize) {
        // receivedData[] now has LX, LY, L2, R2
      }
      receiving = false;
    } else if (receiving && index < arraySize) {
      receivedData[index++] = (int8_t)b;  // store byte
    }
  }
}

float PIDControl(int error) {
  currentTime = millis();
  int deltaT = (currentTime - previousTime);
  if (deltaT <= 0) {
    deltaT = 1;
  }
  derivative = (error - previousError) / (deltaT);
  PID = kp * error + kd * derivative;
  previousError = error;
  previousTime = currentTime;
  PID = constrain(PID, -maxPWM, maxPWM);
  if (abs(PID) <= 1) {
    PID = 0;
  }
  return PID;
}

// void printPS() {
//   Serial.print("LX : ");
//   Serial.print(receivedData[0]);
//   Serial.print("   ||   LY : ");
//   Serial.print(receivedData[1]);
//   Serial.print("   ||   L2 : ");
//   Serial.print(receivedData[2]);
//   Serial.print("   ||   R2 : ");
//   Serial.println(receivedData[3]);
// }

// void printEq() {
//   Serial.print("ANGLE : ");
//   Serial.print(currentAngle);
//   Serial.print("   ||   wLF : ");
//   Serial.print(wLF);
//   Serial.print("   ||   wRF : ");
//   Serial.print(wRF);
//   Serial.print("   ||   wLR : ");
//   Serial.print(wLR);
//   Serial.print("   ||   wRR : ");
//   Serial.println(wRR);
// }