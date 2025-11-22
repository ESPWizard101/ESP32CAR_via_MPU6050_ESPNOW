# ESP-NOW Tilt-Controlled Car 
### Overview: 
Controller ESP32 uses MPU6050 tilt to send speed/angle via ESP-NOW to car ESP32 for motor/servo control.

### Hardware:
Two ESP32s; Controller: MPU6050 (I2C), LED (GPIO2); Car: motor driver (17/16/21/23), servo (GPIO15).

### Software:
Arduino IDE, esp_now/WiFi/ESP32Servo/Wire libraries.
