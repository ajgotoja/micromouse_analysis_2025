#ifndef ROBOT_H
#define ROBOT_H

#include <Adafruit_MPU6050.h>
#include <Adafruit_Sensor.h>


class Robot {
public:
    Robot();
    bool init();
    bool moveForward(int distance, int velocity);
    bool moveAngular(int angle);
    int leftSensor();
    int rightSensor();
    int frontSensors();
    void setInitMove(bool value);
    void rideStop();

private:
    Adafruit_MPU6050 mpu;
    float offsetRot;
    float actualAngle;
    float lastAngle;
    bool initMove;
    int sensorValues[4];
    int IRSensor(int IREmitterPin, int IRReceiverPin);
    int sideSensorsCheck();
    void forwardVelocity(float velocityOfMotorA, float velocityOfMotorB);
    void angularVelocity(float angularValue);
};

void updateEncoderA();
void updateEncoderB();

#endif
