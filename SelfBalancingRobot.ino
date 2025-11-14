#include "I2Cdev.h"
#include <PID_v1.h>
#include "MPU6050_6Axis_MotionApps20.h"
#include <Wire.h>
#include "MPU6050_Calibration.h"

// L298N motor driver pin connections
const int ENA = 3;
const int IN1 = 4;
const int IN2 = 5;
const int ENB = 6;
const int IN3 = 7;
const int IN4 = 8;

// MPU6050 interrupt pin connection
const int INT = 2;

// Define M_PI if not defined
#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

// MPU6050 DMP Settings & Variables
MPU6050 mpu;
bool dmpReady = false;
uint8_t mpuIntStatus;
uint8_t devStatus;
uint16_t packetSize;
uint16_t fifoCount;
uint8_t fifoBuffer[64];
Quaternion q;
VectorFloat gravity;
float ypr[3];
volatile bool mpuInterrupt = false;

// PID Settings & Variables
double input, output;
double setpoint = 180;
double Kp = 30.0, Ki = 100.0, Kd = 2.0;
PID pid(&input, &output, &setpoint, Kp, Ki, Kd, DIRECT);
double fallLimit = 25;

// Interrupt Function
void dmpDataReady() {
  mpuInterrupt = true;
}

MPU6050Calibration mpuCalibration; //callibration

void setup() {
  Serial.begin(115200);
  Wire.begin();

  // Set motor driver pins as outputs
  pinMode(ENA, OUTPUT);
  pinMode(IN1, OUTPUT);
  pinMode(IN2, OUTPUT);
  pinMode(ENB, OUTPUT);
  pinMode(IN3, OUTPUT);
  pinMode(IN4, OUTPUT);

  // Stop motors initially
  stopMotors();

  // MPU setup
  mpu.initialize();
  devStatus = mpu.dmpInitialize();

  // Calibrate MPU6050
  mpuCalibration.runCalibrationWithLED(13);
  mpu.setXGyroOffset(mpuCalibration.getGyroXOffset);
  mpu.setYGyroOffset(mpuCalibration.getGyroYOffset);
  mpu.setZGyroOffset(mpuCalibration.getGyroZOffset);
  mpu.setZAccelOffset(mpuCalibration.getAccelZOffset);

  if (devStatus == 0) {
    mpu.setDMPEnabled(true);
    attachInterrupt(digitalPinToInterrupt(INT), dmpDataReady, RISING);
    mpuIntStatus = mpu.getIntStatus();
    dmpReady = true;
    packetSize = mpu.dmpGetFIFOPacketSize();

    pid.SetMode(AUTOMATIC);
    pid.SetSampleTime(10);
    pid.SetOutputLimits(-255, 255);

    Serial.println("DMP ready. Balancing mode engaged.");
  }
  else {
    Serial.print("DMP Initialization failed (code ");
    Serial.print(devStatus);
    Serial.println(")");
  }
}

void loop() {
  // Check if DMP initialized
  if (!dmpReady) return;

  // Wait for MPU interrupt and data
  fifoCount = mpu.getFIFOCount();
  if (!mpuInterrupt && fifoCount < packetSize) return;

  mpuInterrupt = false;
  mpuIntStatus = mpu.getIntStatus();
  fifoCount = mpu.getFIFOCount();

  if ((mpuIntStatus & 0x10) || fifoCount == 1024) {
    mpu.resetFIFO();
    Serial.println("FIFO overflow!");
    return;
  }

  if (mpuIntStatus & 0x02) {
    while (fifoCount < packetSize) fifoCount = mpu.getFIFOCount();
    mpu.getFIFOBytes(fifoBuffer, packetSize);
    fifoCount -= packetSize;

    mpu.dmpGetQuaternion(&q, fifoBuffer);
    mpu.dmpGetGravity(&gravity, &q);
    mpu.dmpGetYawPitchRoll(ypr, &q, &gravity);

    input = ypr[1] * 180 / M_PI + 180;

    pid.Compute();

    Serial.print("Pitch: "); Serial.print(input);
    Serial.print(" | Output: "); Serial.println(output);

    if (abs(input - setpoint) < fallLimit) {
      if (output > 0) moveForward(constrain(abs(output), 100, 255));
      else if (output < 0) moveBackward(constrain(abs(output), 100, 255));
      else stopMotors();
    }
    else {
      stopMotors();
    }
  }
}

// Motor Functions
void moveForward(int power) {
  analogWrite(ENA, power);
  digitalWrite(IN1, HIGH);
  digitalWrite(IN2, LOW);

  analogWrite(ENB, power);
  digitalWrite(IN3, HIGH);
  digitalWrite(IN4, LOW);
}

void moveBackward(int power) {
  analogWrite(ENA, power);
  digitalWrite(IN1, LOW);
  digitalWrite(IN2, HIGH);

  analogWrite(ENB, power);
  digitalWrite(IN3, LOW);
  digitalWrite(IN4, HIGH);
}

void stopMotors() {
  analogWrite(ENA, 0);
  digitalWrite(IN1, LOW);
  digitalWrite(IN2, LOW);

  analogWrite(ENB, 0);
  digitalWrite(IN3, LOW);
  digitalWrite(IN4, LOW);
}