#include <Wire.h>
#include <Adafruit_VL53L0X.h>
#include "Adafruit_TCS34725.h"
#include <MPU6050.h>

#include <freertos/FreeRTOS.h> // for parallel computing
#include <freertos/task.h>

// ===== I2C MUX =====
#define TCA_ADDR   0x70 // === PCA9548A mux address ===
#define SDA_PIN    21
#define SCL_PIN    22

// ===== MOTOR PINS =====
#define IN1_PIN    14  // Left motor
#define IN2_PIN    27  // Left motor
#define ENA_PIN    26  // Left motor PWM
#define IN3_PIN    25  // Right motor
#define IN4_PIN    33  // Right motor
#define ENB_PIN    32  // Right motor PWM

// === Encoder pins ===
#define ENC_L_PIN 35
#define ENC_LB_PIN 34
#define ENC_R_PIN 39
#define ENC_RB_PIN 36

void IRAM_ATTR encL();
void IRAM_ATTR encR();


// ===== PID CONTROL VARIABLES =====
float leftSetpoint = 0;    // desired speed in ticks/sec
float rightSetpoint = 0;

float leftIntegral = 0;
float rightIntegral = 0;

float leftPrevError = 0;
float rightPrevError = 0;

// PID constants (start with these, tune later)
const float KP = 3.0;
const float KI = 0.2;
const float KD = 0.1;

// Add these near the top of your file:
const int TICKS_PER_REV    = 1500;
const float WHEEL_RADIUS_M = 0.032f / 2.0f;
const float AXLE_LENGTH_M  = 0.0951f;

// Helper: convert linear m/s → ticks/sec
inline int linSpeedToTicks(float v_mps) {
  // rev/sec = v / (2πr)
  float revs_per_sec = v_mps / (TWO_PI * WHEEL_RADIUS_M);
  return (int)(revs_per_sec * TICKS_PER_REV);
}


volatile long ticksL = 0, ticksR = 0;
volatile int lastEncL = 0, lastEncR = 0;


// ===== OBSTACLE AVOIDANCE =====
const int OBST_THRESH_MM = 100;
const int OBST_WARN_MM = 155;
const int OBST_CRITICAL_MM = 45;
const int SPEED_NORMAL = 200;
const int SPEED_AVOID = 150;
const int SPEED_REVERSE = 160;

// Shared sensor variables — mark as volatile since updated in another task
volatile int leftDist = 8000, centerDist = 8000, rightDist = 8000;
volatile float gx = 0, gy = 0, gz = 0;
volatile uint16_t colorR = 0, colorG = 0, colorB = 0, colorC = 0;

volatile bool newSensorData = false; // Flag to indicate fresh sensor data


unsigned long lastPrintTime = 0;
unsigned int lapCount=0;

// ===== I2C MUX CONTROL =====
void tca(uint8_t channel) {
  Wire.beginTransmission(TCA_ADDR);
  Wire.write(1 << channel);
  Wire.endTransmission();
}

// Sensors
Adafruit_VL53L0X tof1, tof2, tof3;// === VL53L0X sensors ===
Adafruit_TCS34725 tcs(TCS34725_INTEGRATIONTIME_50MS, TCS34725_GAIN_4X); // === TCS34725 color sensor ===
MPU6050 mpu;

// ===== MOTOR CONTROL =====
void setupMotors() {
  pinMode(IN1_PIN, OUTPUT);
  pinMode(IN2_PIN, OUTPUT);
  pinMode(IN3_PIN, OUTPUT);
  pinMode(IN4_PIN, OUTPUT);
  pinMode(ENA_PIN, OUTPUT);
  pinMode(ENB_PIN, OUTPUT);
  stopMotors();
  Serial.println("Motors initialized");
}

void stopMotors() {
  digitalWrite(IN1_PIN, LOW);
  digitalWrite(IN2_PIN, LOW);
  digitalWrite(IN3_PIN, LOW);
  digitalWrite(IN4_PIN, LOW);
  analogWrite(ENA_PIN, 0);
  analogWrite(ENB_PIN, 0);
}

void drive(int leftSpeed, int rightSpeed) {
  leftSpeed = constrain(leftSpeed, -255, 255);
  rightSpeed = constrain(rightSpeed, -255, 255);
  
  // LEFT MOTOR
  if (leftSpeed > 0) {
    digitalWrite(IN1_PIN, LOW);
    digitalWrite(IN2_PIN, HIGH);
  } else if (leftSpeed < 0) {
    digitalWrite(IN1_PIN, HIGH);
    digitalWrite(IN2_PIN, LOW);
  } else {
    digitalWrite(IN1_PIN, LOW);
    digitalWrite(IN2_PIN, LOW);
  }
  analogWrite(ENA_PIN, abs(leftSpeed));
  
  // RIGHT MOTOR
  if (rightSpeed > 0) {
    digitalWrite(IN3_PIN, LOW);
    digitalWrite(IN4_PIN, HIGH);
  } else if (rightSpeed < 0) {
    digitalWrite(IN3_PIN, HIGH);
    digitalWrite(IN4_PIN, LOW);
  } else {
    digitalWrite(IN3_PIN, LOW);
    digitalWrite(IN4_PIN, LOW);
  }
  analogWrite(ENB_PIN, abs(rightSpeed));
}

void updatePID() {
  static long prevTicksL = 0;
  static long prevTicksR = 0;
  static unsigned long lastTime = millis();

  unsigned long now = millis();
  float dt = (now - lastTime) / 1000.0;
  lastTime = now;

  long currTicksL, currTicksR;

  noInterrupts();
  currTicksL = ticksL;
  currTicksR = ticksR;
  interrupts();

  // Compute current speed
  float speedL = (currTicksL - prevTicksL) / dt;
  float speedR = (currTicksR - prevTicksR) / dt;

  prevTicksL = currTicksL;
  prevTicksR = currTicksR;

  // PID calculations
  float errorL = leftSetpoint - speedL;
  leftIntegral += errorL * dt;
  float derivativeL = (errorL - leftPrevError) / dt;
  leftPrevError = errorL;
  float outputL = KP * errorL + KI * leftIntegral + KD * derivativeL;

  float errorR = rightSetpoint - speedR;
  rightIntegral += errorR * dt;
  float derivativeR = (errorR - rightPrevError) / dt;
  rightPrevError = errorR;
  float outputR = KP * errorR + KI * rightIntegral + KD * derivativeR;

  Serial.printf("SetL:%.1f SetR:%.1f | SpdL:%.1f SpdR:%.1f | OutL:%.1f OutR:%.1f\n",
      leftSetpoint, rightSetpoint, speedL, speedR, outputL, outputR);

  // Convert to PWM
  int pwmL = constrain(outputL, -255, 255);
  int pwmR = constrain(outputR, -255, 255);

  drive(pwmL, pwmR);
}



// // ===== SENSOR READING =====
// void readAllToF(int &leftDist, int &centerDist, int &rightDist) {
//   VL53L0X_RangingMeasurementData_t m;
  
//   // Left sensor
//   tca(2);
//   tof1.rangingTest(&m, false);
//   leftDist = (m.RangeStatus != 4) ? m.RangeMilliMeter : 8000;
  
//   // Center sensor
//   tca(1);
//   tof2.rangingTest(&m, false);
//   centerDist = (m.RangeStatus != 4) ? m.RangeMilliMeter : 8000;
  
//   // Right sensor
//   tca(0);
//   tof3.rangingTest(&m, false);
//   rightDist = (m.RangeStatus != 4) ? m.RangeMilliMeter : 8000;
// }

// ===== OBSTACLE AVOIDANCE =====
void avoidAndGo(int leftDist, int centerDist, int rightDist) {
  // Define your base speeds (m/s)
  const float V_NORMAL  = 0.20f;   // 0.20 m/s forward
  const float V_AVOID   = 0.12f;   // 0.12 m/s when avoiding
  const float V_REVERSE = -0.10f;  // -0.10 m/s backward

  // Emergency reverse: back up slightly and turn
  if (centerDist < OBST_CRITICAL_MM ||
      leftDist   < OBST_CRITICAL_MM ||
      rightDist  < OBST_CRITICAL_MM) {

    if (leftDist < rightDist) {
      // pivot right while reversing
      leftSetpoint  = linSpeedToTicks(V_REVERSE);
      rightSetpoint = linSpeedToTicks(V_REVERSE * 0.5f);
    }
    else if (rightDist < leftDist) {
      // pivot left while reversing
      leftSetpoint  = linSpeedToTicks(V_REVERSE * 0.5f);
      rightSetpoint = linSpeedToTicks(V_REVERSE);
    }
    else {
      // straight reverse
      leftSetpoint  = linSpeedToTicks(V_REVERSE);
      rightSetpoint = linSpeedToTicks(V_REVERSE);
    }
    Serial.println("EMERGENCY REVERSE");
    return;
  }

  // Warning‑level: slow down and steer
  if (centerDist < OBST_WARN_MM) {
    if (leftDist > rightDist) {
      // steer left
      leftSetpoint  = linSpeedToTicks(V_AVOID * 0.5f);
      rightSetpoint = linSpeedToTicks(V_AVOID);
    }
    else {
      // steer right
      leftSetpoint  = linSpeedToTicks(V_AVOID);
      rightSetpoint = linSpeedToTicks(V_AVOID * 0.5f);
    }
    Serial.println("AVOIDING FRONT OBSTACLE");
  }
  else if (leftDist < OBST_WARN_MM) {
    // obstacle on left: turn right
    leftSetpoint  = linSpeedToTicks(V_AVOID);
    rightSetpoint = linSpeedToTicks(V_AVOID * 0.5f);
    Serial.println("AVOIDING LEFT OBSTACLE");
  }
  else if (rightDist < OBST_WARN_MM) {
    // obstacle on right: turn left
    leftSetpoint  = linSpeedToTicks(V_AVOID * 0.5f);
    rightSetpoint = linSpeedToTicks(V_AVOID);
    Serial.println("AVOIDING RIGHT OBSTACLE");
  }
  else {
    // no obstacles: go normal speed
    leftSetpoint  = linSpeedToTicks(V_NORMAL);
    rightSetpoint = linSpeedToTicks(V_NORMAL);
  }
}
void SensorTask(void *param) {
  const uint8_t muxChannels[] = {2, 1, 0}; // ToF sensors: Left, Center, Right
  int currentChannel = 0;
  int colorReadCounter = 0;  // For infrequent color sensor reads
  VL53L0X_RangingMeasurementData_t m;

  while (true) {
    // Read ToF sensor on current channel
    uint8_t ch = muxChannels[currentChannel];
    tca(ch);

    switch (ch) {
      case 2:
        tof1.rangingTest(&m, false);
        leftDist = (m.RangeStatus != 4) ? m.RangeMilliMeter : 8000;
        break;
      case 1:
        tof2.rangingTest(&m, false);
        centerDist = (m.RangeStatus != 4) ? m.RangeMilliMeter : 8000;
        break;
      case 0:
        tof3.rangingTest(&m, false);
        rightDist = (m.RangeStatus != 4) ? m.RangeMilliMeter : 8000;
        break;
    }

    currentChannel = (currentChannel + 1) % 3;

    // Read IMU gyro data every loop
    mpu.readGyro();
    Vector gyro = mpu.readNormalizeGyro();

    gx = gyro.XAxis;
    gy = gyro.YAxis;
    gz = gyro.ZAxis;

    // Read color sensor every ~500 ms (e.g. every 33 loops × 15 ms delay)
    if (++colorReadCounter >= 33) {
      colorReadCounter = 0;
      tca(3); // MUX channel for color sensor
      uint16_t r, g, b, c;
      tcs.getRawData(&r, &g, &b, &c);
      colorR = r;
      colorG = g;
      colorB = b;
      colorC = c;
    }

    newSensorData = true; // Mark new data ready

    vTaskDelay(15 / portTICK_PERIOD_MS); // Delay ~15 ms to avoid busy loop
  }
}

// for red line dtection 
bool isRedLineDetected(uint16_t r, uint16_t g, uint16_t b, uint16_t c) {
  // Normalize RGB by clear channel to reduce lighting effect
  if (c == 0) return false; // Avoid division by zero
  
  float normR = (float)r / c;
  float normG = (float)g / c;
  float normB = (float)b / c;

  // Simple red detection condition:
  // Red should be significantly higher than green and blue
  // Tune thresholds experimentally for your environment

  if (normR > 0.5 && normG < 0.3 && normB < 0.3) {
    return true;  // Red line detected
  }
  return false;
}

void printEncoderReadings() {
  long left, right;

  // Safely copy volatile variables
  noInterrupts();  // Disable interrupts temporarily
  left = ticksL;
  right = ticksR;
  interrupts();    // Re-enable interrupts

  Serial.print("Left ticks: ");
  Serial.print(left);
  Serial.print(" | Right ticks: ");
  Serial.println(right);
}

void setup_encoders(){
  pinMode(ENC_L_PIN, INPUT);
  pinMode(ENC_LB_PIN, INPUT);
  pinMode(ENC_R_PIN, INPUT);
  pinMode(ENC_RB_PIN, INPUT);

}



// ===== SETUP =====
void setup() {
  Serial.begin(115200);
  Serial.println("===== OBSTACLE AVOIDANCE ROBOT =====");
  
  // Initialize I2C
  Wire.begin(SDA_PIN, SCL_PIN);
  Wire.setClock(400000);
  
  // Setup motors
  setupMotors();

  //setup the encoders
  setup_encoders(); 
  
  // Initialize sensors
  Serial.println("Initializing sensors...");
  
  tca(0); 
  if (tof1.begin()) Serial.println("Left ToF: OK");
  else Serial.println("Left ToF: FAILED");
  
  tca(1); 
  if (tof2.begin()) Serial.println("Center ToF: OK");
  else Serial.println("Center ToF: FAILED");
  
  tca(2); 
  if (tof3.begin()) Serial.println("Right ToF: OK");
  else Serial.println("Right ToF: FAILED");

    // === TCS34725 init ===
  tca(3);
  if (!tcs.begin()) while (1) Serial.println(" TCS34725 fail");
  Serial.println("TCS34725 initialized");

  //MPU 6050 initilazation 
  Serial.println("Initializing MPU6050...");

  // === MPU6050 init ===
  if (!mpu.begin(MPU6050_SCALE_2000DPS, MPU6050_RANGE_2G)) { // Sets gyroscope sensitivity to ±2000°/s (degrees per second) and Sets accelerometer sensitivity to ±2g (g = acceleration due to gravity
    Serial.println(" MPU6050 init failed");
    while (1);
  } else {
    mpu.calibrateGyro();
    mpu.setThreshold(3);
    Serial.println(" MPU6050 initialized");
  }
  attachInterrupt(digitalPinToInterrupt(ENC_L_PIN), encL, CHANGE);
  attachInterrupt(digitalPinToInterrupt(ENC_LB_PIN), encL, CHANGE);
  attachInterrupt(digitalPinToInterrupt(ENC_R_PIN), encR, CHANGE);
  attachInterrupt(digitalPinToInterrupt(ENC_RB_PIN), encR, CHANGE);

 
  Serial.println("===== STARTING OBSTACLE AVOIDANCE =====");

  //creates a new task (thread) and pins it to a specific CPU core.
  xTaskCreatePinnedToCore(SensorTask, "SensorTask", 8192, NULL, 1, NULL, 0);


}

// ===== MAIN LOOP =====
void loop() {
  if (newSensorData) {
    newSensorData = false;  // Reset flag

    // Print sensor readings for debug
    Serial.printf("Distances L:%d C:%d R:%d\n", leftDist, centerDist, rightDist);
    Serial.printf("Gyro X:%.2f Y:%.2f Z:%.2f\n", gx, gy, gz);
    Serial.printf("Color R:%d G:%d B:%d C:%d\n", colorR, colorG, colorB, colorC);
  

    // Use distance sensors for obstacle avoidance
    
    avoidAndGo(leftDist, centerDist, rightDist);

    // lap detection logic here using color sensor values
    if (isRedLineDetected(colorR, colorG, colorB, colorC)) {
      lapCount++;
      Serial.printf("Red line detected! %d",lapCount);
      vTaskDelay(5 / portTICK_PERIOD_MS);;
     
    }
    

  }

  updatePID(); // update the PID values

  if (millis() - lastPrintTime > 100) { // Print every 100 ms
    printEncoderReadings();
    lastPrintTime = millis();
  }

  
  vTaskDelay(100 / portTICK_PERIOD_MS);  // Small delay to avoid flooding serial and CPU
}
