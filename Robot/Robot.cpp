#include "Robot.h"
#include <Arduino.h>

#define MOTOR_A_PWM_PIN 32
#define MOTOR_A_IN1_PIN 33
#define MOTOR_A_IN2_PIN 25
#define MOTOR_A_ENC1_PIN 26
#define MOTOR_A_ENC2_PIN 27
#define MOTOR_B_PWM_PIN 2
#define MOTOR_B_IN1_PIN 16
#define MOTOR_B_IN2_PIN 4
#define MOTOR_B_ENC1_PIN 14
#define MOTOR_B_ENC2_PIN 12
#define MOTOR_STANDBY_PIN 13
#define BUTTON_PIN 23
#define LED_PIN 15
#define IR_EMITTER1_PIN 19
#define IR_EMITTER2_PIN 18
#define IR_EMITTER3_PIN 5
#define IR_EMITTER4_PIN 17
#define IR_RECEIVER1_PIN 36
#define IR_RECEIVER2_PIN 39
#define IR_RECEIVER3_PIN 34
#define IR_RECEIVER4_PIN 35
#define RESOLUTION 0.3723
#define PWM_A_CHANNEL 0
#define PWM_B_CHANNEL 1
#define PWM_FREQ 1000
#define PWM_RES 8
#define LEFT_WALL_DISTANCE 1500
#define RIGHT_WALL_DISTANCE 1500

#define LEFT_AVG_DISTANCE 1600
#define RIGHT_AVG_DISTANCE 1600
#define RIGHT_MAX_DISTANCE 3600
#define LEFT_MAX_DISTANCE 3600
#define LEFT_LOWER_DISTANCE 1500
#define LEFT_UPPER_DISTANCE 1800
#define RIGHT_LOWER_DISTANCE 1500
#define RIGHT_UPPER_DISTANCE 1800

volatile long encoderCountMotorA;
volatile long encoderCountMotorB;
volatile long lastEncoderCountMotorA;
volatile long lastEncoderCountMotorB;
volatile long lastEncodedMotorA;
volatile long lastEncodedMotorB;
unsigned long lastTime;
//float actualDistance;
//float periodTime;
int countDiff;
float v1, v2;

Robot::Robot() : offsetRot(0.0), actualAngle(0.0), lastAngle(0.0), initMove(1)
                 {}

bool Robot::init() {
    if (!mpu.begin())
        return false;
    mpu.setAccelerometerRange(MPU6050_RANGE_8_G);
    mpu.setGyroRange(MPU6050_RANGE_500_DEG);
    mpu.setFilterBandwidth(MPU6050_BAND_21_HZ);
    // define inputs/outputs
    pinMode(MOTOR_A_PWM_PIN, OUTPUT);
    pinMode(MOTOR_A_IN1_PIN, OUTPUT);
    pinMode(MOTOR_A_IN2_PIN, OUTPUT);
    digitalWrite(MOTOR_A_IN1_PIN, LOW);
    digitalWrite(MOTOR_A_IN2_PIN, LOW);
    pinMode(MOTOR_A_ENC1_PIN, INPUT);
    pinMode(MOTOR_A_ENC2_PIN, INPUT);
    pinMode(MOTOR_B_PWM_PIN, OUTPUT);
    pinMode(MOTOR_B_IN1_PIN, OUTPUT);
    pinMode(MOTOR_B_IN2_PIN, OUTPUT);
    digitalWrite(MOTOR_B_IN1_PIN, LOW);
    digitalWrite(MOTOR_B_IN2_PIN, LOW);
    pinMode(MOTOR_B_ENC1_PIN, INPUT);
    pinMode(MOTOR_B_ENC2_PIN, INPUT);
    pinMode(MOTOR_STANDBY_PIN, OUTPUT);
    digitalWrite(MOTOR_STANDBY_PIN, LOW);
    pinMode(IR_EMITTER1_PIN, OUTPUT);
    pinMode(IR_EMITTER2_PIN, OUTPUT);
    pinMode(IR_EMITTER3_PIN, OUTPUT);
    pinMode(IR_EMITTER4_PIN, OUTPUT);
    pinMode(IR_RECEIVER1_PIN, INPUT);
    pinMode(IR_RECEIVER2_PIN, INPUT);
    pinMode(IR_RECEIVER3_PIN, INPUT);
    pinMode(IR_RECEIVER4_PIN, INPUT);
    // define pwm
    ledcAttach(MOTOR_A_PWM_PIN, PWM_FREQ, PWM_RES);
    ledcAttach(MOTOR_B_PWM_PIN, PWM_FREQ, PWM_RES);
    ledcWrite(MOTOR_A_PWM_PIN, 0);
    ledcWrite(MOTOR_B_PWM_PIN, 0);
    // define interrupts
    attachInterrupt(digitalPinToInterrupt(MOTOR_A_ENC1_PIN), updateEncoderA, CHANGE);
    attachInterrupt(digitalPinToInterrupt(MOTOR_A_ENC2_PIN), updateEncoderA, CHANGE);
    attachInterrupt(digitalPinToInterrupt(MOTOR_B_ENC1_PIN), updateEncoderB, CHANGE);
    attachInterrupt(digitalPinToInterrupt(MOTOR_B_ENC2_PIN), updateEncoderB, CHANGE);
    return true;
}

bool Robot::moveForward(int distance, int velocity) {
    float wallGainLeft;
    float wallGainRight;
    float encoderGainLeft = 0.3;
    float encoderGainRight = 0.3;
    float distanceOfMotorA, distanceOfMotorB;
    float velocityOfMotorA, velocityOfMotorB;
    int frontSensorValue;

    if (initMove) {
        digitalWrite(MOTOR_STANDBY_PIN, HIGH);
        sideSensorsCheck();
        initMove = false;
        countDiff = 0;
        frontSensorValue = 4095;
        encoderCountMotorA = 0;
        encoderCountMotorB = 0;
        lastEncoderCountMotorA = 0;
        lastEncoderCountMotorB = 0;
        lastEncodedMotorA = 0;
        lastEncodedMotorB = 0;
        distanceOfMotorA = 0.0;
        distanceOfMotorB = 0.0;
    }

    sideSensorsCheck();
    if(sensorValues[3] < RIGHT_MAX_DISTANCE) { // stabilize to right wall
        if(sensorValues[3] < RIGHT_LOWER_DISTANCE) { 
            //digitalWrite(LED_PIN, HIGH);
            wallGainLeft = (float)sensorValues[3]/(RIGHT_AVG_DISTANCE);
            wallGainRight = 1;
        } else if(sensorValues[3] > RIGHT_UPPER_DISTANCE) {
            wallGainLeft = 1;
            wallGainRight = (float)(RIGHT_MAX_DISTANCE - sensorValues[3])/(RIGHT_MAX_DISTANCE - RIGHT_AVG_DISTANCE);
        } else {
            wallGainLeft = 1;
            wallGainRight = 1;
        }
        wallGainLeft = (wallGainLeft < 0.2) ? 0.2 : wallGainLeft;
        wallGainRight = (wallGainRight < 0.2) ? 0.2 : wallGainRight;
        velocityOfMotorA = velocity * wallGainLeft;
        velocityOfMotorB = velocity * wallGainRight;
    } else if(sensorValues[0] < LEFT_MAX_DISTANCE) { // stabilize to left wall
        if(sensorValues[0] < LEFT_LOWER_DISTANCE) {
            //digitalWrite(LED_PIN, LOW);
            wallGainLeft = 1;
            wallGainRight = (float)sensorValues[0]/(LEFT_AVG_DISTANCE);
        } else if(sensorValues[0] > LEFT_UPPER_DISTANCE) {
            wallGainLeft = (float)(LEFT_MAX_DISTANCE - sensorValues[0])/(LEFT_MAX_DISTANCE - LEFT_AVG_DISTANCE);
            wallGainRight = 1;
        } else {
            wallGainLeft = 1;
            wallGainRight = 1;
        }
        wallGainLeft = (wallGainLeft < 0.2) ? 0.2 : wallGainLeft;
        wallGainRight = (wallGainRight < 0.2) ? 0.2 : wallGainRight;
        velocityOfMotorA = velocity * wallGainLeft;
        velocityOfMotorB = velocity * wallGainRight;
    } else { // stabilize to encoders values
        float encoderDifference = (encoderCountMotorA - encoderCountMotorB)/2.0;
        if(encoderDifference > 0) {
            velocityOfMotorA = velocity + encoderDifference * encoderGainLeft;
            velocityOfMotorB = velocity - encoderDifference * encoderGainRight;
        } else if(encoderDifference < 0) {
            velocityOfMotorA = velocity - encoderDifference * encoderGainLeft;
            velocityOfMotorB = velocity + encoderDifference * encoderGainRight;
        } else {
            velocityOfMotorA = velocity;
            velocityOfMotorB = velocity;
        }
    }

    distanceOfMotorA = encoderCountMotorA * RESOLUTION;
    distanceOfMotorB = encoderCountMotorB * RESOLUTION;
    float u1 = (distance - distanceOfMotorA)/distance;
    float u2 = (distance - distanceOfMotorB)/distance;
    forwardVelocity(u1*velocityOfMotorA, u2*velocityOfMotorB);

    float actualDistance = max(distanceOfMotorA, distanceOfMotorB);

    if (abs(distance - actualDistance) / distance < 0.02 &&
        abs(lastEncoderCountMotorA - encoderCountMotorA) <= 1 &&
        abs(lastEncoderCountMotorB - encoderCountMotorB) <= 1) {
        countDiff++;
    }

    lastEncoderCountMotorA = encoderCountMotorA;
    lastEncoderCountMotorB = encoderCountMotorB;

    frontSensorValue = frontSensors();

    if (countDiff > 10 || frontSensorValue < 300) {
        rideStop();
        initMove = true;
        return true;
    }
    return false;
}

bool Robot::moveAngular(int angle) {
    float angularGain = 0.75;
    sensors_event_t a, g, temp;

    if (initMove) {
        digitalWrite(MOTOR_STANDBY_PIN, HIGH);
        countDiff = 0;
        initMove = false;
        encoderCountMotorA = 0;
        encoderCountMotorB = 0;
        lastEncoderCountMotorA = 0;
        lastEncoderCountMotorB = 0;
        lastEncodedMotorA = 0;
        lastEncodedMotorB = 0;
        offsetRot = 0.0;
        for (int i = 0; i < 10; i++) {
            mpu.getEvent(&a, &g, &temp);
            offsetRot += g.gyro.z;
        }
        offsetRot = offsetRot / 10;
        actualAngle = 0.0;
        lastAngle = 0.0;
        lastTime = micros();
    }

    float periodTime = (micros() - lastTime) / 1.0e6;
    lastTime = micros();
    mpu.getEvent(&a, &g, &temp);
    double correctRot = g.gyro.z - offsetRot;
    actualAngle = lastAngle + (correctRot * periodTime);
    lastAngle = actualAngle;
    actualAngle = actualAngle * (180.0 / PI);
    float u = (angle - actualAngle) * angularGain;
    angularVelocity(u);

    if (abs((angle - actualAngle) / angle) < 0.08 &&
        abs(lastEncoderCountMotorA - encoderCountMotorA) <= 1 &&
        abs(lastEncoderCountMotorB - encoderCountMotorB) <= 1)
        countDiff++;

    lastEncoderCountMotorA = encoderCountMotorA;
    lastEncoderCountMotorB = encoderCountMotorB;

    if (countDiff > 10) {
        rideStop();
        initMove = true;
        return true;
    }
    return false;
}

void Robot::rideStop() {
    digitalWrite(MOTOR_STANDBY_PIN, LOW);
    ledcWrite(MOTOR_A_PWM_PIN, 0);
    ledcWrite(MOTOR_B_PWM_PIN, 0);
    digitalWrite(MOTOR_A_IN1_PIN, LOW);
    digitalWrite(MOTOR_A_IN2_PIN, LOW);
    digitalWrite(MOTOR_B_IN1_PIN, LOW);
    digitalWrite(MOTOR_B_IN2_PIN, LOW);
}

int Robot::sideSensorsCheck() {
    sensorValues[0] = IRSensor(IR_EMITTER1_PIN, IR_RECEIVER1_PIN);
    sensorValues[3] = IRSensor(IR_EMITTER4_PIN, IR_RECEIVER4_PIN);
    return min(sensorValues[0], sensorValues[3]);
}

int Robot::leftSensor() {
    sensorValues[0] = IRSensor(IR_EMITTER1_PIN, IR_RECEIVER1_PIN);
    return(sensorValues[0]);
}

int Robot::rightSensor() {
    sensorValues[3] = IRSensor(IR_EMITTER4_PIN, IR_RECEIVER4_PIN);
    return (sensorValues[3]);
}

int Robot::frontSensors() {
    sensorValues[1] = IRSensor(IR_EMITTER2_PIN, IR_RECEIVER2_PIN);
    sensorValues[2] = IRSensor(IR_EMITTER3_PIN, IR_RECEIVER3_PIN);
    return min(sensorValues[1], sensorValues[2]);
}

int Robot::IRSensor(int IREmitterPin, int IRReceiverPin) {
    digitalWrite(IREmitterPin, HIGH);
    delayMicroseconds(10);
    int sensorValue = analogRead(IRReceiverPin);
    digitalWrite(IREmitterPin, LOW);
    return sensorValue;
}

void Robot::setInitMove(bool value) {
    initMove = value;
}

void Robot::forwardVelocity(float velocityOfMotorA, float velocityOfMotorB) {
    if (velocityOfMotorA > 0) {
        // Motor A (left side)
        // Motor A Forward
        digitalWrite(MOTOR_A_IN1_PIN, LOW);
        digitalWrite(MOTOR_A_IN2_PIN, HIGH);
    } else if (velocityOfMotorA < 0) {
        // Motor A Backward
        digitalWrite(MOTOR_A_IN1_PIN, HIGH);
        digitalWrite(MOTOR_A_IN2_PIN, LOW);
    } else {
        // Motor A Stop
        digitalWrite(MOTOR_A_IN1_PIN, LOW);
        digitalWrite(MOTOR_A_IN2_PIN, LOW);
    }
    if (velocityOfMotorB > 0) {
        // Motor B (right side)
        // Motor B Forward
        digitalWrite(MOTOR_B_IN1_PIN, HIGH);
        digitalWrite(MOTOR_B_IN2_PIN, LOW);
    } else if (velocityOfMotorB < 0) {
        // Motor B Backward
        digitalWrite(MOTOR_B_IN1_PIN, LOW);
        digitalWrite(MOTOR_B_IN2_PIN, HIGH);
    } else {
        // Motor B Stop
        digitalWrite(MOTOR_B_IN1_PIN, LOW);
        digitalWrite(MOTOR_B_IN2_PIN, LOW);
    }

    velocityOfMotorA = abs(velocityOfMotorA) > 100 ? 100 : abs(velocityOfMotorA);
    velocityOfMotorB = abs(velocityOfMotorB) > 100 ? 100 : abs(velocityOfMotorB);
    velocityOfMotorA = map(velocityOfMotorA, 0, 100, 40, 130);
    velocityOfMotorB = map(velocityOfMotorB, 0, 100, 40, 130);
    ledcWrite(MOTOR_A_PWM_PIN, velocityOfMotorA);
    ledcWrite(MOTOR_B_PWM_PIN, velocityOfMotorB);
}

void Robot::angularVelocity(float angularValue) {
    // Motor A and Motor B velocity prescaler (depends on which way robot is turning)
  if (angularValue < 0) {
    // Robot turns right
    // Motor A (left) Forward
    digitalWrite(MOTOR_A_IN1_PIN, LOW);
    digitalWrite(MOTOR_A_IN2_PIN, HIGH);
    // Motor B (right) Backward
    digitalWrite(MOTOR_B_IN1_PIN, LOW);
    digitalWrite(MOTOR_B_IN2_PIN, HIGH);
  } else if (angularValue > 0) {
    // Robot turns left
    // Motor A (left) Backward
    digitalWrite(MOTOR_A_IN1_PIN, HIGH);
    digitalWrite(MOTOR_A_IN2_PIN, LOW);
    // Motor B (right) Forward
    digitalWrite(MOTOR_B_IN1_PIN, HIGH);
    digitalWrite(MOTOR_B_IN2_PIN, LOW);
  } else {
    // Motor A and B are stopped
    digitalWrite(MOTOR_A_IN1_PIN, LOW);
    digitalWrite(MOTOR_A_IN2_PIN, LOW);
    digitalWrite(MOTOR_B_IN1_PIN, LOW);
    digitalWrite(MOTOR_B_IN2_PIN, LOW);
  }
  // Set PWM for Motor A and Motor B
  angularValue = abs(angularValue) > 100 ? 100 : abs(angularValue);
  angularValue = map(angularValue, 0, 100, 30, 125);
  ledcWrite(MOTOR_A_PWM_PIN, angularValue);
  ledcWrite(MOTOR_B_PWM_PIN, angularValue);
}

void updateEncoderA() {
  int MSB = digitalRead(MOTOR_A_ENC1_PIN); //MSB = most significant bit
  int LSB = digitalRead(MOTOR_A_ENC2_PIN); //LSB = least significant bit

  int encoded = (MSB << 1) | LSB; //converting the 2 pin value to single number
  int sum = (lastEncodedMotorA << 2) | encoded; //adding it to the previous encoded value

  if (sum == 0b1110 || sum == 0b0111 || sum == 0b0001 || sum == 0b1000) encoderCountMotorA--;
  if (sum == 0b1101 || sum == 0b0100 || sum == 0b0010 || sum == 0b1011) encoderCountMotorA++;

  lastEncodedMotorA = encoded;
}

void updateEncoderB() {
  int MSB = digitalRead(MOTOR_B_ENC1_PIN); //MSB = most significant bit
  int LSB = digitalRead(MOTOR_B_ENC2_PIN); //LSB = least significant bit

  int encoded = (MSB << 1) | LSB; //converting the 2 pin value to single number
  int sum = (lastEncodedMotorB << 2) | encoded; //adding it to the previous encoded value

  if (sum == 0b1101 || sum == 0b0100 || sum == 0b0010 || sum == 0b1011) encoderCountMotorB--;
  if (sum == 0b1110 || sum == 0b0111 || sum == 0b0001 || sum == 0b1000) encoderCountMotorB++;

  lastEncodedMotorB = encoded;
}
