# ESP32 Autonomous wall following  Robot with Multi-Sensor System & Parallel Processing

This project is an **ESP32-based autonomous robot** equipped with:
- **VL53L0X ToF distance sensors** for obstacle detection
- **TCS34725 color sensor** for color recognition
- **MPU6050 gyroscope & accelerometer** for orientation and stability
- **L298N motor driver** for precise motor control
- **I2C multiplexer (TCA9548A)** for managing multiple I2C devices

The robot uses **parallel processing** with **FreeRTOS** to improve performance:
- **Core 0**: Continuously reads and processes sensor data (distance, color, orientation)
- **Core 1**: Executes decision-making algorithms and drives motors in real time

---

## 🚀 Features

- **Real-Time Obstacle Detection**  
  Uses multiple VL53L0X sensors via an I2C multiplexer for 360° coverage.
- **Color-Based Navigation**  
  Detects and reacts to specific colors using the TCS34725 sensor.
- **Stability & Orientation Control**  
  Reads gyroscope/accelerometer data from the MPU6050.
- **Parallel Processing for Efficiency**  
  Sensor reading and decision-making run independently on separate CPU cores.
- **Scalable Sensor Integration**  
  Easily add more I2C devices without address conflicts.
- **Smooth Motor Control**  
  Powered by TB6612FNG dual-channel motor driver.

---

## 🛠 Hardware Used

| Component | Purpose |
|-----------|---------|
| **ESP32** | Main microcontroller with dual-core capability |
| **VL53L0X** | Time-of-Flight sensors for distance measurement |
| **TCS34725** | Color sensor |
| **MPU6050** | Gyroscope and accelerometer |
| **TCA9548A** | I2C multiplexer |
| **L298N** | Motor driver |
| **N20 Motors (6V)** | Robot propulsion |

---

![WhatsApp Image 2025-08-13 at 20 07 54_be233f61](https://github.com/user-attachments/assets/20d8451a-f229-4ff6-9308-12f8387db0d5)



