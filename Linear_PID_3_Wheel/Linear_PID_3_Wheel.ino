// Teensy / Arduino
#define TEENSY 13

// Encoder Setup
#define offset 0.200
#define T      0.200
#define R      0.125


#include <Encoder.h>
Encoder FEnc(5, 6);
Encoder LEnc(7, 8);
Encoder REnc(9, 10);

long FCount = 0, LCount = 0, RCount = 0;
long prevFCount = 0, prevLCount = 0, prevRCount = 0;
double dF, dL, dR;
double CPR = 8192;

// Position
double xPosition = 0, yPosition = 0, thetaPosition = 0;
double delX, delY, delTheta;
double changeInX, changeInY, changeInTheta;


// BTS7960 Motor Setup
#include <BTS7960.h>
#define maxPWM 50

BTS7960 FW(12, 11);
BTS7960 LW(9, 10);
BTS7960 RW(5, 6);

// Constants for wheel equations
#define sqrt3by2     0.8660254038
#define minus1by2   -0.5000
#define constVector  1
#define L            1

// PS4 data
#define arraySize 4
int8_t receivedData[arraySize] = {0};

int16_t wFW = 0, wLW = 0, wRW = 0;
int16_t Vx = 0, Vy = 0, VxG = 0, VyG = 0, omega = 0;

// PID variables
double currentAngularTime = 0, previousAngularTime = 0;
double angularError = 0, previousAngularError = 0;
double angularDerivative = 0;
double angularKp = 8.0, angularDd = 68;
double angularPID = 0;
double targetAngle = 0;

// Linear PID
double linearKd, linearKp;
double linearDerivative;
double xTargetPosition = 0, yTargetPosition = 0;
double xPositionError = 0, yPositionError = 0;
double xPrevError = 0, yPrevError = 0;
double xCorrection = 0, yCorrection = 0;


// BNO055 IMU Setup
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

  //Encoder Readings
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

  // Odometry calculations
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

  // Final position
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

    omega = AngularPIDControl(error);

    previousError = error;
    previousTime = currentTime;
  } 
  // else {
  //   targetAngle = currentAngle;
  // }

  //Linear PID
  xPositionError = xTargetPosition - xPosition;
  yPositionError = yTargetPosition - yPosition;

  if(abs(xPositionError) < 0.05){
    xCorrection = LinearPIDControl(xPositionError);
  } else{
    xTargetPosition = xPosition;
  }

  if(abs(yPositionError) < 0.05){
    yCorrection = LinearPIDControl(yPositionError);
  } else{
    yTargetPosition = yPosition;
  }

  //Adding correction to velocities
  VxG = VxG + xCorrection;
  VyG = VyG + yCorrection;

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

float AngularPIDControl(int error) {
  currentAngularTime = millis();
  int deltaT = (currentAngularTime - previousAngularTime);
  if (deltaT <= 0) {
    deltaT = 1;
  }
  angularDerivative = (AngularError - previousAngularError) / (deltaT);
  angularPID = angularKp * error + angularKd * angularDerivative;
  previousAngularError = Angularerror;
  previousAngularTime = currentAngularTime;
  angularPID = constrain(PID, -maxPWM, maxPWM);
  if (abs(angularPID) <= 1) {
    angularPID = 0;
  }
  return angularPID;
}

double LinearPIDControl(int error) {
  currentLinearTime = millis();
  int deltaT = (currentLinearTime - previousLinearTime);
  if (deltaT <= 0) {
    deltaT = 1;
  }
  linearDerivative = (linearError - previousLinearError) / (deltaT);
  linearPID = linearKp * linearError + d * linearDerivative;

  previousLinearError = linearError;
  previousLinearTime = currentLinearTime;
  
  linearPID = constrain(linearPID, -maxPWM, maxPWM);
  if (abs(linearPID) <= 1) {
    PID = 0;
  }
  return linearPID;
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