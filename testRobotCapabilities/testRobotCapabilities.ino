#include <Robot.h>
#include <Wire.h>
#include <BluetoothSerial.h>

#define BUTTON_PIN 23
#define LED_PIN 15

const int motorVelocity = 50;
const int distanceToReach = 240;
const int angleToReach = 90;
bool buttonState = HIGH;
int flaga = 1;
int task = 0;
char receivedMsg;

BluetoothSerial SerialBT;
Robot robot;

void setup() {
  pinMode(BUTTON_PIN, INPUT_PULLUP);
  pinMode(LED_PIN, OUTPUT);
  digitalWrite(LED_PIN, HIGH);
  delay(200);
  digitalWrite(LED_PIN, LOW);

  SerialBT.begin("ESP32");

  if (!robot.init()) {
    SerialBT.println("Initialization of robot failed");
    while (1)
      ;
  }
}

void loop() {
  if (SerialBT.available()) {
    receivedMsg = SerialBT.read();
    if (receivedMsg == '1') {
      task = 1;
      SerialBT.println(">Move forward<");
    } else if (receivedMsg == '2') {
      task = 2;
      SerialBT.println(">Turn left<");
    } else if (receivedMsg == '3') {
      task = 3;
      SerialBT.println(">Turn right<");
    } else if (receivedMsg == '4') {
      task = 4;
      SerialBT.println(">Infrared sensors<");
    } else if (receivedMsg == '5') {
      task = 0;
      SerialBT.println(">Stop<");
    }
    flaga = 1;
    digitalWrite(LED_PIN, LOW);
  }

  switch (task) {
    case 0:
      robot.rideStop();
      robot.setInitMove(1);
      digitalWrite(LED_PIN, LOW);
      if (flaga && SerialBT.connected()) {
        flaga = 0;
        SerialBT.println("Choose operation:\n1. Go forward\n2. Turn left\n3. Turn right\n4. Infrared sensors\n5. Stop");
        delay(200);
      }
      break;
    case 1:
      if (robot.moveForward(distanceToReach, motorVelocity)) {
        task = 0;
        SerialBT.print("Finished move");
      }
      break;
    case 2:
      if (robot.moveAngular(angleToReach)) {
        task = 0;
        SerialBT.print("Finished left turn");
      }
      break;
    case 3:
      if (robot.moveAngular(angleToReach * (-1))) {
        task = 0;
        SerialBT.print("Finished right turn");
      }
      break;
    case 4:
      SerialBT.println("Left: " + (String)robot.leftSensor());
      SerialBT.println("Front minimum: " + (String)robot.frontSensors());
      SerialBT.println("Right: " + (String)robot.rightSensor());
      SerialBT.println();
      task = 0;
      delay(100);
      break;
  }
}