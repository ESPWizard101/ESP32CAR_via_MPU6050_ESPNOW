// Code for the Car ESP32 (with motors and servo)

// This code receives speed/angle commands via ESP-NOW and controls the motors/servo.
#include <esp_now.h>
#include <WiFi.h>
#include <ESP32Servo.h>

// Motor pins for the DRI0044 MOTOR DRIVER
const int MOTOR_A_IA = 17;
const int MOTOR_A_IB = 16;
const int MOTOR_B_IA = 21;
const int MOTOR_B_IB = 23;
const int SERVO_PIN = 15;

// Calibration factors
const float FORWARD_FACTOR = 1.2f;
const float BACKWARD_FACTOR = 0.9f;

// Data structure to receive
typedef struct struct_message {
  int speed;  // -100 to 100
  int angle;  // 50 to 140
} struct_message;

struct_message receivedData;

// Globals
Servo steeringServo;
int currentSpeed = 0;
int currentAngle = 95;

// Callback when data received 
void OnDataRecv(const esp_now_recv_info_t *recv_info, const uint8_t *incomingData, int len) {
  
  memcpy(&receivedData, incomingData, sizeof(receivedData));
  setMotorSpeed(receivedData.speed);
  setSteeringAngle(receivedData.angle);
  Serial.printf("Received: Speed %d, Angle %d\n", receivedData.speed, receivedData.angle);
}

void setup() {
  Serial.begin(115200);

  // Print MAC for controller to use
  WiFi.mode(WIFI_STA);
  Serial.print("Car MAC Address: ");
  Serial.println(WiFi.macAddress()); 

  // Initialize motors
  pinMode(MOTOR_A_IA, OUTPUT);
  pinMode(MOTOR_A_IB, OUTPUT);
  pinMode(MOTOR_B_IA, OUTPUT);
  pinMode(MOTOR_B_IB, OUTPUT);

  // Initialize servo
  ESP32PWM::allocateTimer(0);
  steeringServo.setPeriodHertz(50);
  steeringServo.attach(SERVO_PIN, 500, 2400);
  steeringServo.write(currentAngle);

  // Initialize ESP-NOW
  if (esp_now_init() != ESP_OK) {
    Serial.println("ESP-NOW init failed");
    return;
  }

  // Register receive callback
  esp_err_t regResult = esp_now_register_recv_cb(OnDataRecv);
  if (regResult != ESP_OK) {
    Serial.printf("Recv CB registration failed: %d\n", regResult);
    return;
  }

  Serial.println("Car ready");
}

void loop() {
 
  delay(100);
}

// Set motor speed (same as the WiFi project)
void setMotorSpeed(int speed) {
  speed = constrain(speed, -100, 100);
  currentSpeed = speed;
  
  int pwmValue = map(abs(speed), 0, 100, 0, 255);
  
  if (speed > 0) {
    int calibratedPwm = min(255, static_cast<int>(pwmValue * FORWARD_FACTOR));
    analogWrite(MOTOR_A_IA, calibratedPwm);
    digitalWrite(MOTOR_A_IB, LOW);
    analogWrite(MOTOR_B_IA, calibratedPwm);
    digitalWrite(MOTOR_B_IB, LOW);
  } else if (speed < 0) {
    int calibratedPwm = min(255, static_cast<int>(pwmValue * BACKWARD_FACTOR));
    digitalWrite(MOTOR_A_IA, LOW);
    analogWrite(MOTOR_A_IB, calibratedPwm);
    digitalWrite(MOTOR_B_IA, LOW);
    analogWrite(MOTOR_B_IB, calibratedPwm);
  } else {
    digitalWrite(MOTOR_A_IA, LOW);
    digitalWrite(MOTOR_A_IB, LOW);
    digitalWrite(MOTOR_B_IA, LOW);
    digitalWrite(MOTOR_B_IB, LOW);
  }
}

// Set steering angle (same as the WiFi project)
void setSteeringAngle(int angle) {
  angle = constrain(angle, 50, 140);
  currentAngle = angle;
  steeringServo.write(angle);
}