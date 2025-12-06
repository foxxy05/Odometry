// ESP32
#include <PS4Controller.h>
#define buffer 10
#define arraySize 4 
int8_t data[arraySize] = { 0 }; //lx ly l2 r2
int8_t lx = 0, ly = 0;
uint8_t l2 = 0, r2 = 0;

void setup() {
  // put your setup code here, to run once:
  Serial.begin(115200);
  PS4.begin();
  Serial.println("PS4 Ready!");
  PS4.attach(receivePS);

  // Setting-up UART communication between ESP32 and Arduino
  Serial2.begin(9600, SERIAL_8N1, 16, 17);
  Serial.println("UART2 is active!");
  delay(1000);
}

void loop() {
  if (!PS4.isConnected()) {
    data[0] = data[1] = data[2] = data[3] = 0;
  }

  Serial2.write(0xFF);              // Start marker
  Serial2.write((uint8_t*)data, 4); // Send all 4 bytes
  Serial2.write(0xFE);              // End marker

  printPS();
  delay(10);
}

void receivePS() {
  if (PS4.isConnected()) {
    lx = PS4.LStickX();
    ly = PS4.LStickY();
    l2 = PS4.L2Value();
    r2 = PS4.R2Value();

    if (abs(lx) < buffer) lx = 0;
    if (abs(ly) < buffer) ly = 0;
    // if (l2 < (buffer / 2)) l2 = 0;
    // if (r2 < (buffer / 2)) r2 = 0;

    l2 = map(l2, 0, 255, 0, 127);
    r2 = map(r2, 0, 255, 0, 127);

    data[0] = lx;
    data[1] = ly;
    data[2] = l2;
    data[3] = r2;
  }
}

void printPS() {
  Serial.print("LX : ");
  Serial.print(data[0]);
  Serial.print("   ||    LY : ");
  Serial.print(data[1]);
  Serial.print("   ||   L2 : ");
  Serial.print(data[2]);
  // Serial.print(l2);
  Serial.print("   ||   R2 : ");
  Serial.println(data[3]);
  // Serial.println(r2);

  // Serial.print("   ||   Button : ");
  // Serial.println(data[4]);
}