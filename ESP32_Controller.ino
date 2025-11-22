// Code for the Controller ESP32 (with MPU6050 and LED)

// This code reads tilt from MPU6050 accelerometer and sends speed/angle commands to the car ESP32 via ESP-NOW.


#include <Wire.h>
#include <esp_now.h>
#include <WiFi.h>

//The mac address of the other esp32 which will use later in order to connect the 2 ESP32s via ESP_NOW
uint8_t carMAC[] = {0xXX, 0xXX, 0xXX, 0xXX, 0xXX, 0xXX};  

// Data structure to send
typedef struct struct_message {
  int speed;  // -100 to 100
  int angle;  // 50 to 140
} struct_message;

struct_message dataToSend;

// MPU6050 address and variables
#define MPU_ADDR 0x68
int16_t AcX, AcY, AcZ;

// LED pin
const int LED_PIN = 2;

void setup() {
  Serial.begin(115200);
  pinMode(LED_PIN, OUTPUT);
  digitalWrite(LED_PIN, LOW);  // Start off

  // Initialize MPU6050 with I2C
  Wire.begin();
  Wire.beginTransmission(MPU_ADDR);
  Wire.write(0x6B);  // PWR_MGMT_1 register
  Wire.write(0);     // Wake up MPU
  Wire.endTransmission(true);

  // Initialize ESP-NOW
  WiFi.mode(WIFI_STA);
  if (esp_now_init() != ESP_OK) {
    Serial.println("ESP-NOW init failed");
    return;
  }

  // Add car as peer
  esp_now_peer_info_t peerInfo = {};
  memcpy(peerInfo.peer_addr, carMAC, 6);
  peerInfo.channel = 0;
  peerInfo.encrypt = false;
  if (esp_now_add_peer(&peerInfo) != ESP_OK) {
    Serial.println("Failed to add peer");
    return;
  }

  digitalWrite(LED_PIN, HIGH);  // Setup complete, LED on
  Serial.println("Controller ready");
}

void loop() {
  // Read accelerometer data
  Wire.beginTransmission(MPU_ADDR);
  Wire.write(0x3B);  // Start at ACCEL_XOUT_H
  Wire.endTransmission(false);
  Wire.requestFrom(MPU_ADDR, 6, true);   
  AcX = Wire.read() << 8 | Wire.read();  // X
  AcY = Wire.read() << 8 | Wire.read();  // Y
  

  // Map tilt to controls 
  dataToSend.speed = constrain(map(AcY, -8000, 8000, -100, 100), -100, 100);  // Forward/back
  dataToSend.angle = constrain(95 + map(AcX, -8000, 8000, -45, 45), 50, 140);  // Left/right (center 95)

  // Send data
  esp_err_t result = esp_now_send(carMAC, (uint8_t *)&dataToSend, sizeof(dataToSend));
  if (result == ESP_OK) {
    Serial.printf("Sent: Speed %d, Angle %d\n", dataToSend.speed, dataToSend.angle);
  } else {
    Serial.println("Send failed");
  }

  delay(100);  
}