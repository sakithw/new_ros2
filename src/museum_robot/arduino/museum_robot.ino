// ==========================================
// 🏛️ MUSEUM GUIDE ROBOT — ARDUINO MAIN
// Board  : Arduino Mega 2560
// Comms  : Serial2 (TX2/RX2 pins 16/17) → Raspberry Pi via level shifter
// IMU    : MPU6050 via Adafruit library
// Motors : BTS7960 H-Bridge (x2)
// Encoders: Hall-effect, single-channel
// Ultrasonic: HC-SR04 front-facing
// ==========================================

#include <Arduino.h>
#include <Wire.h>
#include <Adafruit_MPU6050.h>
#include <Adafruit_Sensor.h>

Adafruit_MPU6050 mpu;

#define LEFT_ENC  18
#define RIGHT_ENC 19
#define L_EN_R 22
#define L_EN_L 23
#define L_RPWM 12
#define L_LPWM 11
#define R_EN_R 24
#define R_EN_L 25
#define R_RPWM 10
#define R_LPWM  9
#define TRIG_PIN 32
#define ECHO_PIN 33

const float WHEEL_DIAMETER_CM  = 12.0;
const float TRACK_WIDTH_CM     = 30.0;
const int   PULSES_PER_REV     = 20;
const float WHEEL_CIRCUM_CM    = WHEEL_DIAMETER_CM * PI;
const float TICKS_PER_CM       = (float)PULSES_PER_REV / WHEEL_CIRCUM_CM;
const int   DEFAULT_SPEED      = 100;
const int   TURN_SPEED         = 100;
const float Kp_straight        = 6.0;
const unsigned long DEBOUNCE_US = 2000;

volatile long left_count  = 0;
volatile long right_count = 0;
volatile int  left_dir    = 1;
volatile int  right_dir   = 1;
volatile unsigned long last_left_pulse  = 0;
volatile unsigned long last_right_pulse = 0;

float current_yaw   = 0.0;
unsigned long last_imu_time = 0;

// Ultrasonic state
static unsigned long last_us_send = 0;

void left_encoder_isr() {
  unsigned long now = micros();
  if (now - last_left_pulse > DEBOUNCE_US) {
    left_count += left_dir;
    last_left_pulse = now;
  }
}

void right_encoder_isr() {
  unsigned long now = micros();
  if (now - last_right_pulse > DEBOUNCE_US) {
    right_count += right_dir;
    last_right_pulse = now;
  }
}

// Returns front distance in cm. Non-blocking via short pulseIn timeout.
float read_ultrasonic_cm() {
  digitalWrite(TRIG_PIN, LOW);
  delayMicroseconds(2);
  digitalWrite(TRIG_PIN, HIGH);
  delayMicroseconds(10);
  digitalWrite(TRIG_PIN, LOW);
  // 23500 µs timeout ≈ 400 cm max range
  long duration = pulseIn(ECHO_PIN, HIGH, 23500);
  if (duration == 0) return 400.0;
  return duration * 0.01715;  // µs → cm (speed of sound / 2)
}

void send_telemetry() {
  static unsigned long last_odom_send = 0;
  unsigned long now = millis();

  // ODOM at 20 Hz
  if (now - last_odom_send >= 50) {
    noInterrupts();
    long l = left_count;
    long r = right_count;
    interrupts();
    Serial2.print("ODOM:");
    Serial2.print(l);   Serial2.print(",");
    Serial2.print(r);   Serial2.print(",");
    Serial2.println(current_yaw);
    last_odom_send = now;
  }

  // Ultrasonic at 5 Hz (every 200 ms)
  if (now - last_us_send >= 200) {
    float dist_cm = read_ultrasonic_cm();
    Serial2.print("US:");
    Serial2.println(dist_cm, 1);
    last_us_send = now;
  }
}

void updateIMU() {
  sensors_event_t a, g, temp;
  mpu.getEvent(&a, &g, &temp);
  unsigned long now = millis();
  float dt = (now - last_imu_time) / 1000.0;
  last_imu_time = now;
  float gz = g.gyro.z;
  if (abs(gz) < 0.05) gz = 0.0;
  current_yaw += (gz * RAD_TO_DEG) * dt;
}

void setMotors(int leftSpeed, int rightSpeed) {
  left_dir  = (leftSpeed  >= 0) ? 1 : -1;
  right_dir = (rightSpeed >= 0) ? 1 : -1;
  if (leftSpeed > 0) {
    analogWrite(L_RPWM, leftSpeed);  analogWrite(L_LPWM, 0);
  } else if (leftSpeed < 0) {
    analogWrite(L_RPWM, 0);          analogWrite(L_LPWM, abs(leftSpeed));
  } else {
    analogWrite(L_RPWM, 0);          analogWrite(L_LPWM, 0);
  }
  if (rightSpeed > 0) {
    analogWrite(R_RPWM, rightSpeed); analogWrite(R_LPWM, 0);
  } else if (rightSpeed < 0) {
    analogWrite(R_RPWM, 0);          analogWrite(R_LPWM, abs(rightSpeed));
  } else {
    analogWrite(R_RPWM, 0);          analogWrite(R_LPWM, 0);
  }
}

void stop_all_motors() { setMotors(0, 0); }

void travel_distance(float distance_cm, bool forward) {
  if (distance_cm <= 0) { Serial2.println("DONE"); return; }
  long pulses_needed = (long)(distance_cm * TICKS_PER_CM);
  noInterrupts();
  long start_l = left_count;
  long start_r = right_count;
  interrupts();
  float start_yaw   = current_yaw;
  long  ramp_pulses = (long)(15.0 * TICKS_PER_CM);

  while (true) {
    // Check for S command
    while (Serial2.available() > 0) {
      char c = Serial2.read();
      if (c == 'S' || c == 's') {
        stop_all_motors();
        Serial2.println("STOPPED");
        return;
      }
    }

    noInterrupts();
    long moved_l = left_count  - start_l;
    long moved_r = right_count - start_r;
    interrupts();
    long avg = (moved_l + moved_r) / 2;
    if (avg >= pulses_needed) break;

    updateIMU();
    send_telemetry();

    int base_speed;
    if (avg < ramp_pulses) {
      base_speed = map(avg, 0, ramp_pulses, 80, DEFAULT_SPEED);
    } else if ((pulses_needed - avg) < ramp_pulses) {
      base_speed = map((pulses_needed - avg), 0, ramp_pulses, 80, DEFAULT_SPEED);
    } else {
      base_speed = DEFAULT_SPEED;
    }
    base_speed = constrain(base_speed, 80, DEFAULT_SPEED);
    if (!forward) base_speed = -base_speed;

    float heading_error = current_yaw - start_yaw;
    int   correction    = (int)(heading_error * Kp_straight);
    int left_speed  = base_speed + correction;
    int right_speed = base_speed - correction;
    if (forward) {
      left_speed  = constrain(left_speed,  80, 255);
      right_speed = constrain(right_speed, 80, 255);
    } else {
      left_speed  = constrain(left_speed,  -255, -80);
      right_speed = constrain(right_speed, -255, -80);
    }
    setMotors(left_speed, right_speed);
    delay(2);
  }
  stop_all_motors();
  Serial2.println("DONE");
}

void turn_degrees(float target_degrees) {
  if (abs(target_degrees) < 1.0) { Serial2.println("DONE"); return; }
  float start_yaw  = current_yaw;
  bool  turn_right = (target_degrees > 0);

  while (true) {
    if (Serial2.available() > 0) {
      char c = Serial2.peek();
      if (c == 'S' || c == 's') {
        Serial2.read();
        stop_all_motors();
        Serial2.println("STOPPED");
        return;
      }
    }

    updateIMU();
    send_telemetry();

    float degrees_turned = current_yaw - start_yaw;
    float error = abs(target_degrees) - abs(degrees_turned);
    if (error <= 0) break;

    int spd = map((int)error, 0, (int)abs(target_degrees), 100, TURN_SPEED);
    spd = constrain(spd, 100, 255);
    setMotors(turn_right ?  spd : -spd,
              turn_right ? -spd :  spd);
    delay(2);
  }
  stop_all_motors();
  delay(500);
  Serial2.println("DONE");
}

void setup() {
  Serial2.begin(115200);

  pinMode(L_EN_R, OUTPUT); pinMode(L_EN_L, OUTPUT);
  pinMode(L_RPWM, OUTPUT); pinMode(L_LPWM, OUTPUT);
  pinMode(R_EN_R, OUTPUT); pinMode(R_EN_L, OUTPUT);
  pinMode(R_RPWM, OUTPUT); pinMode(R_LPWM, OUTPUT);
  digitalWrite(L_EN_R, HIGH); digitalWrite(L_EN_L, HIGH);
  digitalWrite(R_EN_R, HIGH); digitalWrite(R_EN_L, HIGH);
  stop_all_motors();

  pinMode(LEFT_ENC,  INPUT_PULLUP);
  pinMode(RIGHT_ENC, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(LEFT_ENC),  left_encoder_isr,  RISING);
  attachInterrupt(digitalPinToInterrupt(RIGHT_ENC), right_encoder_isr, RISING);

  pinMode(TRIG_PIN, OUTPUT);
  pinMode(ECHO_PIN, INPUT);

  Wire.begin();
  if (!mpu.begin()) {
    Serial2.println("ERROR: IMU_NOT_FOUND");
  } else {
    mpu.setGyroRange(MPU6050_RANGE_500_DEG);
    mpu.setFilterBandwidth(MPU6050_BAND_21_HZ);
    last_imu_time = millis();
    Serial2.println("INFO: IMU_OK");
  }
  Serial2.println("READY");
}

void loop() {
  updateIMU();
  send_telemetry();

  if (Serial2.available() > 0) {
    char cmd = Serial2.read();
    if (cmd == 'S' || cmd == 's') {
      stop_all_motors();
      Serial2.println("STOPPED");
    } else if (cmd == 'F' || cmd == 'f') {
      float val = Serial2.parseFloat();
      travel_distance(val, true);
    } else if (cmd == 'B' || cmd == 'b') {
      float val = Serial2.parseFloat();
      travel_distance(val, false);
    } else if (cmd == 'T' || cmd == 't') {
      float val = Serial2.parseFloat();
      turn_degrees(val);
    }
  }
}
