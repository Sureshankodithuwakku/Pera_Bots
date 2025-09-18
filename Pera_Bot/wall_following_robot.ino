#include <Wire.h>
#include <Adafruit_VL53L0X.h>
#include <Adafruit_TCS34725.h>
#include <MPU6050.h>
#include <algorithm>
#include <math.h>

// ===== I2C MUX =====
#define TCA_ADDR   0x70
#define SDA_PIN    21
#define SCL_PIN    22

// ===== MOTOR PINS =====
#define IN1_PIN    14  // Left motor
#define IN2_PIN    27  // Left motor
#define ENA_PIN    26  // Left motor PWM
#define IN3_PIN    25  // Right motor
#define IN4_PIN    33  // Right motor
#define ENB_PIN    32  // Right motor PWM

// ===== ENCODER PINS =====
#define ENC_L_PIN  35 
#define ENC_LB_PIN 34 
#define ENC_R_PIN  39
#define ENC_RB_PIN 36
const int TICKS_PER_REV = 20;

// ===== PHYSICAL PARAMETERS =====
const float WHEEL_RADIUS_M = 0.016f;     
const float AXLE_LENGTH_M  = 0.0951f;    
const unsigned long LOOP_MS = 64;
const float ROBOT_LENGTH = 0.12f;  // 12cm robot length

// ===== MAPPING PARAMETERS =====
const int MAX_POINTS = 2000;
const int WP_COUNT_DES = 16;
const float MIN_WP_SPACING = 0.10f;
const float WP_RADIUS_M = 0.12f;

// ===== SPEED PARAMETERS =====
const int SPEED_MAP = 150;
const int SPEED_FAST = 250;
const int SPEED_AVOID = 10;
const int SPEED_CORNER = 180;  // Reduced speed for corners

// ===== OBSTACLE AVOIDANCE =====
const int OBST_THRESH_MM = 200;
const int OBST_WARN_MM = 200;
const int OBST_CRITICAL_MM = 80;
const int WALL_FOLLOW_DIST = 80;
const float SENSOR_ANGLE = 30.0 * M_PI / 180.0;  // 30-degree sensor spread

// ===== RED LINE DETECTION =====
const unsigned long LAP_PAUSE_MS = 1500;t
float redThreshNorm = 0;

// ===== PID CONTROL =====
const float WALL_KP = 0.4f;
const float WALL_KD = 0.2f;
float prevWallError = 0;

// ===== GLOBAL VARIABLES =====
volatile long ticksL = 0, ticksR = 0;
volatile int lastEncL = 0, lastEncR = 0;

// Sensors
Adafruit_VL53L0X tof1, tof2, tof3;
Adafruit_TCS34725 tcs(TCS34725_INTEGRATIONTIME_50MS, TCS34725_GAIN_4X);
MPU6050 mpu;

// Path and waypoints
struct Pt { float x, y; };
Pt pathPts[MAX_POINTS];
int pathN = 0;
Pt wps[WP_COUNT_DES];
int wpN = 0;
int tour[WP_COUNT_DES + 2], tourLen = 0, tourIdx = 0;

// Robot state
float poseX = 0, poseY = 0, poseTh = 0;
long prevL = 0, prevR = 0;
bool mapping = true;
int lap = 0;
bool lapPaused = false;
unsigned long pauseStart = 0;
unsigned long lastRedLineTime = 0;
unsigned long lastLoop = 0;

// ===== I2C MUX CONTROL =====
void tca(uint8_t channel) {
  Wire.beginTransmission(TCA_ADDR);
  Wire.write(1 << channel);
  Wire.endTransmission();
}

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

// ===== ENCODER HANDLERS =====
void IRAM_ATTR encL() {
  int msb = digitalRead(ENC_L_PIN), lsb = digitalRead(ENC_LB_PIN);
  int code = (msb << 1) | lsb;
  int sum = (lastEncL << 2) | code;
  if (sum == 0b1101 || sum == 0b0100 || sum == 0b0010 || sum == 0b1011) ticksL++;
  if (sum == 0b1110 || sum == 0b0111 || sum == 0b0001 || sum == 0b1000) ticksL--;
  lastEncL = code;
}

void IRAM_ATTR encR() {
  int msb = digitalRead(ENC_R_PIN), lsb = digitalRead(ENC_RB_PIN);
  int code = (msb << 1) | lsb;
  int sum = (lastEncR << 2) | code;
  if (sum == 0b1101 || sum == 0b0100 || sum == 0b0010 || sum == 0b1011) ticksR++;
  if (sum == 0b1110 || sum == 0b0111 || sum == 0b0001 || sum == 0b1000) ticksR--;
  lastEncR = code;
}

// ===== SENSOR READING =====
void readAllToF(int &leftDist, int &centerDist, int &rightDist) {
  VL53L0X_RangingMeasurementData_t m;
  
  // Left sensor
  tca(0);
  tof1.rangingTest(&m, false);
  leftDist = (m.RangeStatus != 4) ? m.RangeMilliMeter : 8000;
  
  // Center sensor
  tca(1);
  tof2.rangingTest(&m, false);
  centerDist = (m.RangeStatus != 4) ? m.RangeMilliMeter : 8000;
  
  // Right sensor
  tca(2);
  tof3.rangingTest(&m, false);
  rightDist = (m.RangeStatus != 4) ? m.RangeMilliMeter : 8000;
}

// ===== POSE ESTIMATION =====
void updatePose() {
  long currentL = ticksL, currentR = ticksR;
  long deltaL = currentL - prevL, deltaR = currentR - prevR;
  prevL = currentL; prevR = currentR;

  float rotationsL = (float)deltaL / TICKS_PER_REV;
  float rotationsR = (float)deltaR / TICKS_PER_REV;
  float distanceL = rotationsL * 2 * M_PI * WHEEL_RADIUS_M;
  float distanceR = rotationsR * 2 * M_PI * WHEEL_RADIUS_M;
  
  float forwardDistance = (distanceL + distanceR) / 2.0f;
  float angularChange = (distanceR - distanceL) / AXLE_LENGTH_M;
  
  poseTh += angularChange;
  while (poseTh >= 2 * M_PI) poseTh -= 2 * M_PI;
  while (poseTh < 0) poseTh += 2 * M_PI;
  
  poseX += forwardDistance * cos(poseTh);
  poseY += forwardDistance * sin(poseTh);
}

// ===== RED LINE DETECTION =====
bool detectRedLine() {
  if (millis() - lastRedLineTime < 3000) return false;
  
  tca(3);
  uint16_t r, g, b, c;
  tcs.getRawData(&r, &g, &b, &c);
  
  if (c == 0) return false;
  
  float redRatio = (float)r / (float)c;
  bool isRedLine = redRatio > redThreshNorm;
  
  if (isRedLine) {
    lastRedLineTime = millis();
    Serial.printf("RED LINE DETECTED! Ratio: %.3f\n", redRatio);
  }
  
  return isRedLine;
}

void calibrateRed() {
  Serial.println("Calibrating red line detection...");
  float sum = 0;
  int count = 0;
  
  for (int i = 0; i < 50; i++) {
    tca(3);
    uint16_t r, g, b, c;
    tcs.getRawData(&r, &g, &b, &c);
    
    if (c > 0) {
      sum += (float)r / (float)c;
      count++;
    }
    delay(20);
  }
  
  redThreshNorm = (count > 0) ? (sum / count) + 0.08f : 0.35f;
  Serial.printf("Red threshold: %.3f\n", redThreshNorm);
}

// ===== WAYPOINT GENERATION =====
void buildWaypoints() {
  if (pathN == 0) {
    Serial.println("No path recorded!");
    return;
  }
  
  wpN = 0;
  int step = max(1, pathN / WP_COUNT_DES);
  
  for (int i = 0; i < pathN && wpN < WP_COUNT_DES; i += step) {
    bool valid = true;
    for (int j = 0; j < wpN; j++) {
      float dx = pathPts[i].x - wps[j].x;
      float dy = pathPts[i].y - wps[j].y;
      if (sqrt(dx*dx + dy*dy) < MIN_WP_SPACING) {
        valid = false;
        break;
      }
    }
    
    if (valid) {
      wps[wpN] = pathPts[i];
      wpN++;
    }
  }
  
  Serial.printf("Generated %d waypoints\n", wpN);
}

void planTour() {
  tourLen = 0;
  tourIdx = 0;
  
  for (int i = 0; i < wpN; i++) {
    tour[tourLen++] = i;
  }
  
  Serial.printf("Tour planned: %d waypoints\n", tourLen);
}

// ===== IMPROVED WALL FOLLOWING =====
void smoothWallFollow(int leftDist, int centerDist, int rightDist, int &leftSpeed, int &rightSpeed) {
  leftSpeed = rightSpeed = SPEED_MAP;
  
  // Emergency avoidance
  if (centerDist < OBST_CRITICAL_MM) {
    leftSpeed = -SPEED_AVOID;
    rightSpeed = SPEED_AVOID;
    Serial.println("Emergency avoidance");
    return;
  }
  
  // Calculate effective distances with robot geometry
  float effectiveLeft = leftDist / cos(SENSOR_ANGLE) + ROBOT_LENGTH * 0.5f;
  float effectiveRight = rightDist / cos(SENSOR_ANGLE) + ROBOT_LENGTH * 0.5f;
  
  // Corner detection
  bool leftCorner = (effectiveLeft < 150 && centerDist > 300);
  bool rightCorner = (effectiveRight < 150 && centerDist > 300);
  
  if (leftCorner) {
    // Smooth right turn for left corner
    leftSpeed = SPEED_CORNER;
    rightSpeed = SPEED_CORNER * 0.6f;
    Serial.println("Left corner detected");
  } 
  else if (rightCorner) {
    // Smooth left turn for right corner
    leftSpeed = SPEED_CORNER * 0.6f;
    rightSpeed = SPEED_CORNER;
    Serial.println("Right corner detected");
  }
  else {
    // PID-based wall following
    float error = effectiveLeft - WALL_FOLLOW_DIST;
    float derivative = (error - prevWallError) / (LOOP_MS / 1000.0f);
    prevWallError = error;
    
    float control = WALL_KP * error + WALL_KD * derivative;
    control = constrain(control, -50, 50);
    
    leftSpeed = SPEED_MAP - control;
    rightSpeed = SPEED_MAP + control;
  }
  
  // Ensure minimum speeds
  leftSpeed = max(leftSpeed, 80);
  rightSpeed = max(rightSpeed, 80);
}

// ===== NAVIGATION BEHAVIORS =====
void navigateToWaypoint(int &leftSpeed, int &rightSpeed) {
  if (tourIdx >= tourLen) {
    leftSpeed = rightSpeed = 0;
    return;
  }
  
  Pt target = wps[tour[tourIdx]];
  float dx = target.x - poseX;
  float dy = target.y - poseY;
  float distance = sqrt(dx*dx + dy*dy);
  
  if (distance < WP_RADIUS_M) {
    tourIdx++;
    Serial.printf("Waypoint %d reached!\n", tourIdx-1);
    
    if (tourIdx >= tourLen) {
      leftSpeed = rightSpeed = 0;
      return;
    }
    
    target = wps[tour[tourIdx]];
    dx = target.x - poseX;
    dy = target.y - poseY;
    distance = sqrt(dx*dx + dy*dy);
  }
  
  float targetHeading = atan2(dy, dx);
  float headingError = targetHeading - poseTh;
  
  // Normalize angle error
  while (headingError > M_PI) headingError -= 2 * M_PI;
  while (headingError < -M_PI) headingError += 2 * M_PI;
  
  // Dynamic steering based on distance
  float steerGain = 60.0f + 40.0f * (1.0f - min(1.0f, distance/0.5f));
  float steerIntensity = headingError * steerGain;
  steerIntensity = constrain(steerIntensity, -80, 80);
  
  leftSpeed = constrain(SPEED_FAST - steerIntensity, 30, 255);
  rightSpeed = constrain(SPEED_FAST + steerIntensity, 30, 255);
}

bool avoidObstacles(int leftDist, int centerDist, int rightDist, int &leftSpeed, int &rightSpeed) {
  // Check if obstacles are too close
  bool criticalObstacle = (centerDist < OBST_CRITICAL_MM) || 
                          (leftDist < OBST_CRITICAL_MM) || 
                          (rightDist < OBST_CRITICAL_MM);
  
  bool warningObstacle = (centerDist < OBST_THRESH_MM) || 
                         (leftDist < OBST_THRESH_MM) || 
                         (rightDist < OBST_THRESH_MM);
  
  if (!criticalObstacle && !warningObstacle) return false;
  
  Serial.printf("Avoiding obstacles: L:%d C:%d R:%d\n", leftDist, centerDist, rightDist);
  
  // Emergency reverse if very close
  if (criticalObstacle) {
    leftSpeed = rightSpeed = -SPEED_AVOID;
    Serial.println("Emergency reverse");
    return true;
  }
  
  // Choose direction with more space
  if (leftDist > rightDist + 50) {
    leftSpeed = SPEED_AVOID / 2;
    rightSpeed = SPEED_AVOID;
    Serial.println("Turn left");
  } else if (rightDist > leftDist + 50) {
    leftSpeed = SPEED_AVOID;
    rightSpeed = SPEED_AVOID / 2;
    Serial.println("Turn right");
  } else if {
    // Default: turn right
    leftSpeed = SPEED_AVOID;
    rightSpeed = SPEED_AVOID / 2;
  }else if ( rightDist < centerDist && leftDist < centerDist){
    leftSpeed = SPEED_AVOID / 2;
    rightSpeed = SPEED_AVOID;
  }else if ()
  
  return true;
}

// ===== SETUP =====
void setup() {
  Serial.begin(115200);
  Serial.println("===== ESP32 2-WHEEL RACING ROBOT =====");
  
  // Initialize I2C
  Wire.begin(SDA_PIN, SCL_PIN);
  Wire.setClock(400000);
  
  // Setup encoders
  pinMode(ENC_L_PIN, INPUT_PULLUP);
  pinMode(ENC_LB_PIN, INPUT_PULLUP);
  pinMode(ENC_R_PIN, INPUT_PULLUP);
  pinMode(ENC_RB_PIN, INPUT_PULLUP);
  
  attachInterrupt(digitalPinToInterrupt(ENC_L_PIN), encL, CHANGE);
  attachInterrupt(digitalPinToInterrupt(ENC_LB_PIN), encL, CHANGE);
  attachInterrupt(digitalPinToInterrupt(ENC_R_PIN), encR, CHANGE);
  attachInterrupt(digitalPinToInterrupt(ENC_RB_PIN), encR, CHANGE);
  
  // Setup motors
  setupMotors();
  
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
  
  tca(3); 
  if (tcs.begin()) {
    Serial.println("Color sensor: OK");
    calibrateRed();
  } else {
    Serial.println("Color sensor: FAILED");
  }
  
  while (!mpu.begin(MPU6050_SCALE_2000DPS, MPU6050_RANGE_2G)) {
    Serial.println("MPU6050 failed, retrying...");
    delay(1000);
  }
  mpu.calibrateGyro();
  Serial.println("MPU6050: OK");
  
  prevL = ticksL;
  prevR = ticksR;
  
  Serial.println("===== STARTING MAPPING PHASE =====");
  lastLoop = millis();
}

// ===== MAIN LOOP =====
void loop() {
  unsigned long now = millis();
  if (now - lastLoop < LOOP_MS) return;
  lastLoop = now;
  
  updatePose();
  
  int leftDist, centerDist, rightDist;
  readAllToF(leftDist, centerDist, rightDist);
  
  // ===== MAPPING PHASE =====
  if (mapping) {
    if (detectRedLine()) {
      if (!lapPaused) {
        lap++;
        Serial.printf("Mapping lap %d completed\n", lap);
        lapPaused = true;
        pauseStart = now;
        stopMotors();
        
        if (lap >= 2) {
          mapping = false;
          Serial.println("===== MAPPING COMPLETE =====");
          buildWaypoints();
          planTour();
          Serial.println("===== RACING PHASE STARTED =====");
        }
      }
    } else {
      lapPaused = false;
    }
    
    if (lapPaused && (now - pauseStart < LAP_PAUSE_MS)) {
      return;
    }
    lapPaused = false;
    
    if (pathN < MAX_POINTS) {
      pathPts[pathN++] = {poseX, poseY};
    }
    
    int leftSpeed, rightSpeed;
    
    if (avoidObstacles(leftDist, centerDist, rightDist, leftSpeed, rightSpeed)) {
      // Obstacle avoidance active
    } else {
      smoothWallFollow(leftDist, centerDist, rightDist, leftSpeed, rightSpeed);
    }
    
    drive(leftSpeed, rightSpeed);
    return;
  }
  
  // ===== RACING PHASE =====
  if (detectRedLine()) {
    if (!lapPaused) {
      lap++;
      Serial.printf("Racing lap %d completed\n", lap);
      lapPaused = true;
      pauseStart = now;
      
      if (lap >= 5) {
        Serial.println("===== RACE FINISHED =====");
        stopMotors();
        while (true) {
          Serial.println("Race complete. Robot stopped.");
          delay(5000);
        }
      }
    }
  } else {
    lapPaused = false;
  }
  
  if (lapPaused && (now - pauseStart < LAP_PAUSE_MS)) {
    stopMotors();
    return;
  }
  lapPaused = false;
  
  int leftSpeed, rightSpeed;
  
  if (avoidObstacles(leftDist, centerDist, rightDist, leftSpeed, rightSpeed)) {
    // Using obstacle avoidance
  } else if (wpN > 0) {
    navigateToWaypoint(leftSpeed, rightSpeed);
  } else {
    // Fallback to wall following
    smoothWallFollow(leftDist, centerDist, rightDist, leftSpeed, rightSpeed);
  }
  
  drive(leftSpeed, rightSpeed);
  
  static unsigned long lastStatus = 0;
  if (now - lastStatus > 2500) {
    lastStatus = now;
    Serial.printf("Status: Lap %d, Pos(%.2f,%.2f), WP %d/%d\n", 
                  lap, poseX, poseY, tourIdx, tourLen);
  }
}